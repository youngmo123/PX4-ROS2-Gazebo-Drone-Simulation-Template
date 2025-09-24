#!/usr/bin/env python3
"""
MAVSDK Drone Control Server
웹 인터페이스와 MAVSDK를 연결하는 백엔드 서버
"""

import asyncio
import json
import logging
import websockets
from datetime import datetime
from typing import Dict, Any, Optional
import mavsdk
from mavsdk import System
import threading
import time
import http.server
import socketserver
import os
import subprocess
import signal

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 커스텀 HTTP 핸들러
class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/restart':
            self.handle_restart()
        else:
            self.send_error(404, "Not Found")
    
    def handle_restart(self):
        try:
            # Content-Length 읽기
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            # JSON 파싱
            import json
            data = json.loads(post_data.decode('utf-8'))
            
            if data.get('action') == 'restart':
                logger.info("Server restart requested from web interface")
                
                # 재시작 응답 전송
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                
                response = json.dumps({
                    'success': True,
                    'message': 'Server restart initiated'
                })
                self.wfile.write(response.encode('utf-8'))
                
                # 서버 재시작 (백그라운드에서)
                threading.Thread(target=self.restart_server, daemon=True).start()
            else:
                self.send_error(400, "Invalid action")
                
        except Exception as e:
            logger.error(f"Restart request error: {e}")
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def restart_server(self):
        """서버 재시작 함수"""
        try:
            # 현재 프로세스 ID
            current_pid = os.getpid()
            logger.info(f"Restarting server (PID: {current_pid})")
            
            # 재시작 스크립트 실행
            script_path = os.path.join(os.path.dirname(__file__), 'auto_start.sh')
            if os.path.exists(script_path):
                subprocess.Popen(['bash', script_path], 
                               cwd=os.path.dirname(__file__),
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                logger.info("Restart script executed")
            else:
                logger.error("Restart script not found")
            
            # 현재 프로세스 종료
            time.sleep(1)  # 응답 전송 완료 대기
            os.kill(current_pid, signal.SIGTERM)
            
        except Exception as e:
            logger.error(f"Server restart error: {e}")

class DroneController:
    """MAVSDK 드론 제어 클래스"""
    
    def __init__(self):
        self.drone = System()
        self.connected = False
        self.armed = False
        self.in_air = False
        self.max_altitude = 5.0  # 창고 내부 기본 최대 고도 (미터)
        self.telemetry_data = {
            'altitude': 0.0,
            'speed': 0.0,
            'battery': 100.0,
            'gps_sats': 0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'signal_strength': 100.0,
            'position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'flight_mode': 'UNKNOWN',
            'armed': False,
            'in_air': False,
            'max_altitude': self.max_altitude
        }
        self.telemetry_task = None
        self.websocket_clients = set()
        
    async def connect(self, connection_url: str = "udp://:14540") -> bool:
        """드론에 연결"""
        try:
            logger.info(f"Connecting to drone at {connection_url}")
            await self.drone.connect(system_address=connection_url)
            
            # 연결 확인
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    logger.info("Drone connected successfully")
                    await self.start_telemetry()
                    return True
                break
                
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
            
    async def disconnect(self):
        """드론 연결 해제"""
        try:
            self.connected = False
            if self.telemetry_task:
                self.telemetry_task.cancel()
            logger.info("Disconnected from drone")
        except Exception as e:
            logger.error(f"Disconnect error: {e}")
    
    async def arm(self) -> bool:
        """드론 시동"""
        try:
            if not self.connected:
                return False
                
            logger.info("Arming drone...")
            await self.drone.action.arm()
            self.armed = True
            self.telemetry_data['armed'] = True
            logger.info("Drone armed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Arming failed: {e}")
            return False
    
    async def disarm(self) -> bool:
        """드론 시동 해제"""
        try:
            if not self.connected:
                return False
                
            logger.info("Disarming drone...")
            await self.drone.action.disarm()
            self.armed = False
            self.telemetry_data['armed'] = False
            logger.info("Drone disarmed")
            return True
            
        except Exception as e:
            logger.error(f"Disarming failed: {e}")
            return False
    
    async def takeoff(self, altitude: float = 3.0, max_altitude: float = None) -> bool:
        """이륙"""
        try:
            if not self.connected or not self.armed:
                return False
            
            # 고도 제한 확인
            if max_altitude:
                self.max_altitude = max_altitude
                self.telemetry_data['max_altitude'] = self.max_altitude
                
            if altitude > self.max_altitude:
                logger.error(f"Takeoff altitude {altitude}m exceeds max altitude {self.max_altitude}m")
                return False
                
            logger.info(f"Taking off to {altitude}m (max: {self.max_altitude}m)...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            self.in_air = True
            self.telemetry_data['in_air'] = True
            logger.info("Takeoff initiated")
            return True
            
        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            return False
    
    async def land(self) -> bool:
        """착륙"""
        try:
            if not self.connected:
                return False
                
            logger.info("Landing...")
            await self.drone.action.land()
            self.in_air = False
            self.telemetry_data['in_air'] = False
            logger.info("Landing initiated")
            return True
            
        except Exception as e:
            logger.error(f"Landing failed: {e}")
            return False
    
    async def return_to_launch(self) -> bool:
        """자동 복귀"""
        try:
            if not self.connected:
                return False
                
            logger.info("Returning to launch...")
            await self.drone.action.return_to_launch()
            logger.info("Return to launch initiated")
            return True
            
        except Exception as e:
            logger.error(f"RTL failed: {e}")
            return False
    
    async def hold(self) -> bool:
        """위치 유지"""
        try:
            if not self.connected:
                return False
                
            logger.info("Holding position...")
            await self.drone.action.hold()
            logger.info("Position hold activated")
            return True
            
        except Exception as e:
            logger.error(f"Hold failed: {e}")
            return False
    
    async def goto_location(self, latitude: float, longitude: float, altitude: float, max_altitude: float = None) -> bool:
        """지정 위치로 이동"""
        try:
            if not self.connected:
                return False
            
            # 고도 제한 확인
            if max_altitude:
                self.max_altitude = max_altitude
                self.telemetry_data['max_altitude'] = self.max_altitude
                
            if altitude > self.max_altitude:
                logger.error(f"Target altitude {altitude}m exceeds max altitude {self.max_altitude}m")
                return False
                
            logger.info(f"Going to {latitude}, {longitude} at {altitude}m (max: {self.max_altitude}m)")
            await self.drone.action.goto_location(latitude, longitude, altitude, 0.0)
            logger.info("Mission started")
            return True
            
        except Exception as e:
            logger.error(f"Go to failed: {e}")
            return False
    
    async def set_manual_control(self, forward: float, right: float, down: float, yaw: float):
        """수동 제어"""
        try:
            if not self.connected:
                return
            
            # 현재 고도가 최대 고도를 초과하지 않도록 제한
            current_altitude = self.telemetry_data.get('altitude', 0.0)
            if current_altitude >= self.max_altitude and down < 0:  # down < 0 means going up
                logger.warning(f"Altitude limit reached ({self.max_altitude}m). Preventing upward movement.")
                down = 0  # 상승 방지
                
            await self.drone.manual_control.set_manual_control_input(
                forward=forward,
                right=right,
                down=down,
                yaw=yaw
            )
            
        except Exception as e:
            logger.error(f"Manual control failed: {e}")
    
    async def start_telemetry(self):
        """텔레메트리 수집 시작"""
        if self.telemetry_task:
            self.telemetry_task.cancel()
            
        self.telemetry_task = asyncio.create_task(self._collect_telemetry())
    
    async def _collect_telemetry(self):
        """텔레메트리 데이터 수집"""
        try:
            # 고도 정보
            async for position in self.drone.telemetry.position():
                self.telemetry_data['altitude'] = position.relative_altitude_m
                self.telemetry_data['position'] = {
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt': position.absolute_altitude_m
                }
                break
                
            # 속도 정보
            async for velocity in self.drone.telemetry.velocity_ned():
                speed = (velocity.north_m_s**2 + velocity.east_m_s**2 + velocity.down_m_s**2)**0.5
                self.telemetry_data['speed'] = speed
                self.telemetry_data['velocity'] = {
                    'vx': velocity.north_m_s,
                    'vy': velocity.east_m_s,
                    'vz': velocity.down_m_s
                }
                break
                
            # 배터리 정보
            async for battery in self.drone.telemetry.battery():
                self.telemetry_data['battery'] = battery.remaining_percent * 100
                break
                
            # GPS 정보
            async for gps_info in self.drone.telemetry.gps_info():
                self.telemetry_data['gps_sats'] = gps_info.num_satellites
                break
                
            # 자세 정보
            async for attitude in self.drone.telemetry.attitude_euler():
                self.telemetry_data['roll'] = attitude.roll_deg
                self.telemetry_data['pitch'] = attitude.pitch_deg
                self.telemetry_data['yaw'] = attitude.yaw_deg
                break
                
            # 비행 모드
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.telemetry_data['flight_mode'] = str(flight_mode)
                break
                
            # 연결 상태
            async for connection_state in self.drone.core.connection_state():
                self.telemetry_data['connected'] = connection_state.is_connected
                break
                
        except Exception as e:
            logger.error(f"Telemetry collection error: {e}")
    
    def get_telemetry_data(self) -> Dict[str, Any]:
        """현재 텔레메트리 데이터 반환"""
        return self.telemetry_data.copy()

class WebSocketServer:
    """WebSocket 서버 클래스"""
    
    def __init__(self, host: str = "localhost", port: int = 8765, web_port: int = 8000):
        self.host = host
        self.port = port
        self.web_port = web_port
        self.drone_controller = DroneController()
        self.clients = set()
        self.http_server = None
        
    async def register_client(self, websocket):
        """클라이언트 등록"""
        self.clients.add(websocket)
        logger.info(f"Client connected. Total clients: {len(self.clients)}")
        
    async def unregister_client(self, websocket):
        """클라이언트 해제"""
        self.clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(self.clients)}")
    
    async def broadcast_telemetry(self):
        """모든 클라이언트에게 텔레메트리 데이터 브로드캐스트"""
        while True:
            try:
                if self.clients:
                    telemetry_data = self.drone_controller.get_telemetry_data()
                    message = json.dumps({
                        'type': 'telemetry',
                        'data': telemetry_data,
                        'timestamp': datetime.now().isoformat()
                    })
                    
                    # 연결이 끊어진 클라이언트 제거
                    disconnected_clients = set()
                    for client in self.clients:
                        try:
                            await client.send(message)
                        except websockets.exceptions.ConnectionClosed:
                            disconnected_clients.add(client)
                    
                    for client in disconnected_clients:
                        await self.unregister_client(client)
                
                await asyncio.sleep(0.1)  # 10Hz 업데이트
                
            except Exception as e:
                logger.error(f"Broadcast error: {e}")
                await asyncio.sleep(1)
    
    async def handle_client(self, websocket):
        """클라이언트 요청 처리"""
        await self.register_client(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    response = await self.process_command(data)
                    
                    if response:
                        await websocket.send(json.dumps(response))
                        
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': 'Invalid JSON format'
                    }))
                except Exception as e:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': str(e)
                    }))
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister_client(websocket)
    
    async def process_command(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """명령 처리"""
        command = data.get('command')
        params = data.get('params', {})
        
        try:
            if command == 'connect':
                connection_url = params.get('url', 'udp://:14540')
                success = await self.drone_controller.connect(connection_url)
                return {
                    'type': 'response',
                    'command': 'connect',
                    'success': success,
                    'message': 'Connected successfully' if success else 'Connection failed'
                }
                
            elif command == 'disconnect':
                await self.drone_controller.disconnect()
                return {
                    'type': 'response',
                    'command': 'disconnect',
                    'success': True,
                    'message': 'Disconnected'
                }
                
            elif command == 'arm':
                success = await self.drone_controller.arm()
                return {
                    'type': 'response',
                    'command': 'arm',
                    'success': success,
                    'message': 'Armed successfully' if success else 'Arming failed'
                }
                
            elif command == 'disarm':
                success = await self.drone_controller.disarm()
                return {
                    'type': 'response',
                    'command': 'disarm',
                    'success': success,
                    'message': 'Disarmed successfully' if success else 'Disarming failed'
                }
                
            elif command == 'takeoff':
                altitude = params.get('altitude', 3.0)
                max_altitude = params.get('max_altitude')
                success = await self.drone_controller.takeoff(altitude, max_altitude)
                return {
                    'type': 'response',
                    'command': 'takeoff',
                    'success': success,
                    'message': f'Takeoff initiated to {altitude}m' if success else 'Takeoff failed - check altitude limits'
                }
                
            elif command == 'land':
                success = await self.drone_controller.land()
                return {
                    'type': 'response',
                    'command': 'land',
                    'success': success,
                    'message': 'Landing initiated' if success else 'Landing failed'
                }
                
            elif command == 'rtl':
                success = await self.drone_controller.return_to_launch()
                return {
                    'type': 'response',
                    'command': 'rtl',
                    'success': success,
                    'message': 'Return to launch initiated' if success else 'RTL failed'
                }
                
            elif command == 'hold':
                success = await self.drone_controller.hold()
                return {
                    'type': 'response',
                    'command': 'hold',
                    'success': success,
                    'message': 'Position hold activated' if success else 'Hold failed'
                }
                
            elif command == 'goto':
                lat = params.get('latitude')
                lon = params.get('longitude')
                alt = params.get('altitude', 3.0)
                max_altitude = params.get('max_altitude')
                
                if lat is None or lon is None:
                    return {
                        'type': 'error',
                        'message': 'Latitude and longitude are required'
                    }
                
                success = await self.drone_controller.goto_location(lat, lon, alt, max_altitude)
                return {
                    'type': 'response',
                    'command': 'goto',
                    'success': success,
                    'message': f'Mission started to {alt}m' if success else 'Mission failed - check altitude limits'
                }
                
            elif command == 'manual_control':
                forward = params.get('forward', 0.0)
                right = params.get('right', 0.0)
                down = params.get('down', 0.0)
                yaw = params.get('yaw', 0.0)
                
                await self.drone_controller.set_manual_control(forward, right, down, yaw)
                return {
                    'type': 'response',
                    'command': 'manual_control',
                    'success': True,
                    'message': 'Manual control updated'
                }
                
            elif command == 'get_telemetry':
                telemetry_data = self.drone_controller.get_telemetry_data()
                return {
                    'type': 'telemetry',
                    'data': telemetry_data,
                    'timestamp': datetime.now().isoformat()
                }
                
            else:
                return {
                    'type': 'error',
                    'message': f'Unknown command: {command}'
                }
                
        except Exception as e:
            logger.error(f"Command processing error: {e}")
            return {
                'type': 'error',
                'message': str(e)
            }
    
    def start_http_server(self):
        """HTTP 웹 서버 시작"""
        try:
            # 현재 디렉토리를 웹 루트로 설정
            web_dir = os.path.dirname(os.path.abspath(__file__))
            os.chdir(web_dir)
            
            # HTTP 서버 설정 (커스텀 핸들러 사용)
            handler = CustomHTTPRequestHandler
            self.http_server = socketserver.TCPServer((self.host, self.web_port), handler)
            
            logger.info(f"Starting HTTP server on {self.host}:{self.web_port}")
            logger.info(f"Web interface available at: http://{self.host}:{self.web_port}/")
            
            # 별도 스레드에서 HTTP 서버 실행
            http_thread = threading.Thread(target=self.http_server.serve_forever)
            http_thread.daemon = True
            http_thread.start()
            
        except Exception as e:
            logger.error(f"Failed to start HTTP server: {e}")
    
    async def start_server(self):
        """서버 시작"""
        # HTTP 웹 서버 시작
        self.start_http_server()
        
        logger.info(f"Starting WebSocket server on {self.host}:{self.port}")
        
        # 텔레메트리 브로드캐스트 태스크 시작
        broadcast_task = asyncio.create_task(self.broadcast_telemetry())
        
        # WebSocket 서버 시작
        server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port
        )
        
        logger.info("WebSocket server started successfully")
        logger.info(f"📱 Web interface: http://{self.host}:{self.web_port}/drone_control_mavsdk.html")
        logger.info(f"🔌 WebSocket server: ws://{self.host}:{self.port}")
        
        try:
            await server.wait_closed()
        except KeyboardInterrupt:
            logger.info("Shutting down server...")
            broadcast_task.cancel()
            server.close()
            if self.http_server:
                self.http_server.shutdown()
            await server.wait_closed()

def main():
    """메인 함수"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MAVSDK Drone Control Server')
    parser.add_argument('--host', default='localhost', help='Server host')
    parser.add_argument('--port', type=int, default=8765, help='WebSocket server port')
    parser.add_argument('--web-port', type=int, default=8000, help='HTTP web server port')
    parser.add_argument('--connection-url', default='udp://:14540', help='Drone connection URL')
    
    args = parser.parse_args()
    
    # 서버 시작
    server = WebSocketServer(args.host, args.port, args.web_port)
    
    try:
        asyncio.run(server.start_server())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")

if __name__ == "__main__":
    main()
