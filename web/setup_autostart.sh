#!/bin/bash

# 시스템 부팅 시 자동으로 드론 웹 서버를 시작하도록 설정하는 스크립트

echo "🔧 드론 웹 서버 자동 시작 설정 중..."

# 현재 사용자 확인
USER=$(whoami)
echo "👤 현재 사용자: $USER"

# systemd 서비스 파일 생성
SERVICE_FILE="/home/$USER/.config/systemd/user/drone-web-server.service"

# 디렉토리 생성
mkdir -p "/home/$USER/.config/systemd/user"

# 서비스 파일 작성
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Drone Web Server
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/PX4-ROS2-Gazebo-Drone-Simulation-Template/web
ExecStart=/home/$USER/PX4-ROS2-Gazebo-Drone-Simulation-Template/web/auto_start.sh
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
EOF

echo "📄 서비스 파일 생성: $SERVICE_FILE"

# systemd 데몬 리로드
systemctl --user daemon-reload

# 서비스 활성화
systemctl --user enable drone-web-server.service

echo "✅ 자동 시작 설정 완료!"
echo ""
echo "📋 사용 가능한 명령어:"
echo "  시작: systemctl --user start drone-web-server"
echo "  중지: systemctl --user stop drone-web-server"
echo "  상태: systemctl --user status drone-web-server"
echo "  로그: journalctl --user -u drone-web-server -f"
echo ""
echo "🔄 재부팅 후 자동으로 서버가 시작됩니다."
echo "🌐 웹 인터페이스: http://localhost:8000/drone_control_mavsdk.html"

