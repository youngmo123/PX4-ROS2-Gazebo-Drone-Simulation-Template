#!/bin/bash

# 드론 웹 서버 자동 시작 스크립트
# 컴퓨터 부팅 시 자동으로 실행되도록 설정

echo "🚁 드론 웹 서버 자동 시작 중..."

# 웹 디렉토리로 이동
cd /home/youngmo/PX4-ROS2-Gazebo-Drone-Simulation-Template/web

# 의존성 확인 및 설치
echo "📥 의존성 확인 중..."
pip3 install -r requirements.txt > /dev/null 2>&1

# 서버 시작
echo "🚀 드론 웹 서버 시작 중..."
echo "📱 웹 인터페이스: http://localhost:8000/drone_control_mavsdk.html"
echo "🔌 WebSocket 서버: ws://localhost:8765"
echo "📡 드론 연결: udp://:14540 (기본값)"
echo ""
echo "서버가 백그라운드에서 실행됩니다."
echo "종료하려면: pkill -f drone_server.py"
echo ""

# 백그라운드에서 서버 실행
nohup python3 drone_server.py --host localhost --port 8765 --web-port 8000 --connection-url "udp://:14540" > /tmp/drone_server.log 2>&1 &

echo "✅ 드론 웹 서버가 백그라운드에서 시작되었습니다!"
echo "📋 로그 확인: tail -f /tmp/drone_server.log"
echo "🔄 서버 상태 확인: ps aux | grep drone_server"
