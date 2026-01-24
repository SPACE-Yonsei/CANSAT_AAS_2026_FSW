#!/bin/bash
# CANSAT AAS 2026 Flight Software Initial Installation Script
# This script installs all required dependencies and sets up the system

# Note: set -e is not used to allow graceful handling of optional packages

echo "=========================================="
echo "CANSAT AAS 2026 FSW Installation Script"
echo "=========================================="
echo ""

echo "Performing apt update, upgrade"

sudo apt-get update
yes | sudo apt-get -y upgrade

echo ""
echo "Installing basic system packages"
sudo apt-get install -y python3-pip python3-venv python3-dev git

echo ""
echo "Installing Python development tools"
sudo apt install -y --upgrade python3-setuptools

# Optional: Install Python 3.13-dev if available (as mentioned in README)
if apt-cache show python3.13-dev &>/dev/null; then
    echo "Installing python3.13-dev (optional)"
    sudo apt install -y python3.13-dev || echo "Warning: python3.13-dev not available, continuing with default Python"
else
    echo "python3.13-dev not available, using default Python version"
fi

echo ""
echo "Creating virtual environment"
cd ~
python3 -m venv env --system-site-packages

echo ""
echo "Activating virtual environment"
source ~/env/bin/activate

echo ""
echo "Installing pigpio"
pip3 install pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo ""
echo "Installing Adafruit Blinka (do not reboot!)"
cd ~
pip3 install --upgrade adafruit-python-shell
wget -q https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
yes n | sudo -E env PATH=$PATH python3 raspi-blinka.py
rm -f raspi-blinka.py  # Clean up downloaded file

echo ""
echo "Installing Adafruit sensor libraries"
pip3 install adafruit-circuitpython-bmp3xx
pip3 install adafruit-circuitpython-bno055
pip3 install adafruit-circuitpython-motor
pip3 install adafruit-circuitpython-ina23x

echo ""
echo "Installing GPS library (smbus2 for GPS-00177)"
pip3 install smbus2

echo ""
echo "Installing video libraries"
sudo apt install -y ffmpeg
pip3 install opencv-python
sudo apt install -y python3-picamera2
sudo apt install -y libcamera-apps libcamera-tools

echo ""
echo "Installing basic Python modules"
pip3 install numpy==1.26.4

echo ""
echo "Initializing git submodules (if repository is cloned)"
if [ -d "/home/pi/CANSAT_AAS_2026_FSW/.git" ]; then
    cd /home/pi/CANSAT_AAS_2026_FSW
    git submodule init
    git submodule update
    echo "Git submodules initialized"
else
    echo "Warning: Git repository not found at /home/pi/CANSAT_AAS_2026_FSW"
    echo "Please clone the repository and run 'git submodule init && git submodule update' manually"
fi

echo ""
echo "Setting up startup configuration"
# Remove old crontab entry if exists, then add new one
(crontab -l 2>/dev/null | grep -v "@reboot /home/pi/CANSAT_AAS_2026_FSW/startup.sh" || true; echo "@reboot /home/pi/CANSAT_AAS_2026_FSW/startup.sh") | crontab -

# Note: Systemd service can be set up manually if needed
if [ -f "/home/pi/CANSAT_AAS_2026_FSW/setup_systemd_service.sh" ]; then
    echo ""
    echo "Note: Systemd service setup script found."
    echo "      To set up systemd service, run: bash /home/pi/CANSAT_AAS_2026_FSW/setup_systemd_service.sh"
fi

echo ""
echo "=========================================="
echo "Installation completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Configure camera settings in /boot/firmware/config.txt for Zero Spy Cam 160degree:"
echo "   camera_auto_detect=0"
echo "   [all]"
echo "   dtoverlay=ov5647"
echo ""
echo "2. Verify installation by activating venv and running:"
echo "   source ~/env/bin/activate"
echo "   cd /home/pi/CANSAT_AAS_2026_FSW"
echo "   python3 main.py"
echo ""
echo "3. The system will reboot in 10 seconds..."
echo "   (Press Ctrl+C to cancel)"
sleep 10

echo ""
echo "Rebooting system..."
sudo systemctl reboot