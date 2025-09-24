# MAVSDK Drone Control Dashboard

MAVSDK를 사용한 드론 제어 웹 대시보드입니다. 실시간 센서 데이터 모니터링과 드론 제어 기능을 제공합니다.

## 🚀 주요 기능

### 📊 실시간 센서 모니터링
- **고도 (Altitude)**: 상대 고도 및 절대 고도
- **속도 (Speed)**: 3D 속도 벡터
- **배터리 (Battery)**: 배터리 잔량 및 상태
- **GPS**: 위성 수 및 위치 정보
- **자세 (Attitude)**: Roll, Pitch, Yaw 각도
- **신호 강도**: 통신 신호 품질

### 🎮 드론 제어
- **연결 관리**: MAVSDK 연결 및 해제
- **시동 제어**: 드론 시동/해제
- **비행 제어**: 이륙, 착륙, 자동 복귀
- **미션 제어**: 지정 위치 이동, 궤도 비행
- **수동 제어**: 조이스틱을 통한 실시간 제어

### 🎯 미션 계획
- **고도 제한**: 창고 내부 안전 비행을 위한 최대 고도 설정 (기본 5m)
- **위치 지정**: 위도/경도/고도 입력
- **자동 이동**: 지정 위치로 자동 비행
- **궤도 비행**: 특정 지점 주위 궤도 비행
- **안전 제한**: 모든 비행 명령에서 고도 제한 자동 적용

## 📁 파일 구조

```
web/
├── drone_control.html              # 시뮬레이션용 웹 인터페이스
├── drone_control_mavsdk.html      # 실제 MAVSDK 연동 웹 인터페이스
├── drone_server.py                # MAVSDK 백엔드 서버
├── requirements.txt               # Python 의존성
├── start_drone_server.sh         # 서버 시작 스크립트
├── vworld_route.html             # VWorld 라우트 플래너
└── README.md                     # 이 파일
```

## 🛠️ 설치 및 실행

### **방법 1: 수동 실행 (매번 실행 필요)**

#### 1. 의존성 설치 및 서버 시작
```bash
./start_drone_server.sh
```

#### 2. 웹 인터페이스 접속
브라우저에서 다음 URL로 접속:
- **실제 MAVSDK 연동**: `http://localhost:8000/drone_control_mavsdk.html`
- **시뮬레이션 모드**: `http://localhost:8000/drone_control.html`

### **방법 2: 백그라운드 자동 실행**

#### 1. 자동 시작 스크립트 실행
```bash
./auto_start.sh
```

#### 2. 서버 상태 확인
```bash
# 서버 실행 상태 확인
ps aux | grep drone_server

# 로그 확인
tail -f /tmp/drone_server.log
```

#### 3. 서버 종료
```bash
pkill -f drone_server.py
```

### **방법 3: 시스템 부팅 시 자동 시작 (권장)**

#### 1. 자동 시작 설정
```bash
./setup_autostart.sh
```

#### 2. 서비스 관리 명령어
```bash
# 서비스 시작
systemctl --user start drone-web-server

# 서비스 중지
systemctl --user stop drone-web-server

# 서비스 상태 확인
systemctl --user status drone-web-server

# 서비스 로그 확인
journalctl --user -u drone-web-server -f
```

#### 3. 재부팅 후 자동 시작
컴퓨터를 재부팅하면 자동으로 드론 웹 서버가 시작됩니다.

## 🔧 설정

### 연결 설정
- **기본 연결**: `udp://:14540` (PX4 SITL 기본값)
- **직렬 연결**: `serial:///dev/ttyUSB0:57600`
- **TCP 연결**: `tcp://192.168.1.100:5760`

### 서버 설정
```bash
python3 drone_server.py --help
```

옵션:
- `--host`: 서버 호스트 (기본값: localhost)
- `--port`: WebSocket 포트 (기본값: 8765)
- `--connection-url`: 드론 연결 URL

## 🎮 사용법

### 1. 연결
1. 드론 연결 URL 입력
2. "Connect" 버튼 클릭
3. 연결 상태 확인

### 2. 시동
1. "Arm" 버튼으로 드론 시동
2. 시동 상태 확인

### 3. 비행
- **이륙**: "Takeoff" 버튼
- **착륙**: "Land" 버튼
- **자동 복귀**: "RTL" 버튼
- **위치 유지**: "Hold" 버튼

### 4. 미션
1. 최대 고도 설정 (창고 높이에 맞게 조정, 기본 5m)
2. 목표 위치 입력 (위도, 경도, 고도)
3. "Go To" 또는 "Orbit" 버튼 클릭
4. 고도 제한 초과 시 자동으로 명령 거부

### 5. 수동 제어
- **Throttle 조이스틱**: 상하 이동
- **Yaw 조이스틱**: 좌우 회전

## 🔍 문제 해결

### 연결 문제
- 드론이 실행 중인지 확인
- 연결 URL이 올바른지 확인
- 방화벽 설정 확인

### 센서 데이터 문제
- GPS 신호 확인
- 배터리 상태 확인
- 통신 신호 강도 확인

### 웹 인터페이스 문제
- 브라우저 콘솔에서 오류 확인
- WebSocket 연결 상태 확인
- 서버 로그 확인

## 📡 API 참조

### WebSocket 메시지 형식

#### 클라이언트 → 서버
```json
{
  "command": "connect|disconnect|arm|disarm|takeoff|land|rtl|hold|goto|orbit|manual_control",
  "params": {
    "url": "udp://:14540",
    "altitude": 10.0,
    "latitude": 37.5665,
    "longitude": 126.9780,
    "forward": 0.0,
    "right": 0.0,
    "down": 0.0,
    "yaw": 0.0
  }
}
```

#### 서버 → 클라이언트
```json
{
  "type": "telemetry|response|error",
  "data": {
    "altitude": 10.5,
    "speed": 2.3,
    "battery": 85.0,
    "gps_sats": 12,
    "roll": 1.2,
    "pitch": -0.8,
    "yaw": 45.0,
    "signal_strength": 95.0,
    "position": {"lat": 37.5665, "lon": 126.9780, "alt": 100.0},
    "velocity": {"vx": 1.0, "vy": 0.5, "vz": 0.0},
    "flight_mode": "AUTO",
    "armed": true,
    "in_air": true
  },
  "timestamp": "2024-01-01T12:00:00.000Z"
}
```

## 🔒 안전 주의사항

⚠️ **중요**: 실제 드론을 제어할 때는 다음 사항을 준수하세요:

1. **고도 제한**: 창고 내부 비행 시 최대 고도를 적절히 설정 (기본 5m)
2. **안전한 환경**: 넓고 장애물이 없는 공간에서만 비행
3. **배터리 확인**: 충분한 배터리 잔량 확인
4. **GPS 신호**: 충분한 GPS 위성 수 확인
5. **비상 정지**: 비상시 즉시 착륙 또는 RTL 실행
6. **법규 준수**: 해당 지역의 드론 비행 규정 준수
7. **실내 비행**: 창고 내부에서는 GPS 신호가 약할 수 있으므로 주의

## 📝 라이선스

이 프로젝트는 Apache 2.0 라이선스 하에 배포됩니다.

## 🤝 기여

버그 리포트, 기능 요청, 풀 리퀘스트를 환영합니다.

## 📞 지원

문제가 발생하면 다음을 확인하세요:
1. 이 README의 문제 해결 섹션
2. GitHub Issues
3. MAVSDK 공식 문서
