#!/bin/bash
# CANSAT FSW systemd 서비스 설치 스크립트

echo "Installing CANSAT FSW systemd service..."

# 서비스 파일 복사
sudo cp cansat-fsw.service /etc/systemd/system/

# systemd 재로드
sudo systemctl daemon-reload

# 서비스 활성화 (부팅 시 자동 시작)
sudo systemctl enable cansat-fsw.service

echo ""
echo "Service installed and enabled!"
echo ""
echo "Useful commands:"
echo "  Start service:    sudo systemctl start cansat-fsw.service"
echo "  Stop service:     sudo systemctl stop cansat-fsw.service"
echo "  Restart service:  sudo systemctl restart cansat-fsw.service"
echo "  Check status:     sudo systemctl status cansat-fsw.service"
echo "  View logs:        sudo journalctl -u cansat-fsw.service -f"
echo "  Disable service:  sudo systemctl disable cansat-fsw.service"


