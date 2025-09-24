#!/bin/bash

# MAVSDK Drone Control Server 시작 스크립트

echo "🚁 MAVSDK Drone Control Server 시작 중..."

# 의존성 설치 (시스템 환경에 직접 설치)
echo "📥 의존성 설치 중..."
pip3 install -r requirements.txt

# 서버 시작
echo "🚀 MAVSDK 서버 시작 중..."
echo "📱 웹 인터페이스: http://localhost:8000/drone_control_mavsdk.html"
echo "🔌 WebSocket 서버: ws://localhost:8765"
echo "📡 드론 연결: udp://:14540 (기본값)"
echo ""
echo "종료하려면 Ctrl+C를 누르세요."

python3 drone_server.py --host localhost --port 8765 --web-port 8000 --connection-url "udp://:14540"
