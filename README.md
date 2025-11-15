FLIGHT SOFTWARE FOR CANSAT AAS 2026

To make a new app, copy from sample app and replace 'sample' to app name (case sensitive)

# Prerequisities

## 0. Clone directory
Please clone this FSW in /home/pi for smooth automatic initialization! Set the raspberry pi name as pi!
    
    sudo apt update
    sudo apt install git
    ssh-keygen -t rsa -b 4096 -C "jmpark3972@yonsei.ac.kr" # Enter 3번
    cat ~/.ssh/id_rsa.pub # 키 복사, github 세팅 -> SSH and GPG key -> New SSH Key -> Title 아무거나, key 그대로 붙여넣기
    git clone git@github.com:SPACE-Yonsei/CANSAT_AAS_2026_FSW.git

    pi@spacey:~/CANSAT_AAS_2026_FSW $
    python3 -m venv venv # 가상환경 설치
    source venv/bin/activate # 가상환경 실행

    sudo apt update
    sudo apt install python3.13-dev


## 1. Install Adafruit Blinka
Before installing Adafruit modules, follow the steps provided in the link below to install adafruit-blinka
https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

## 2. Install sensor libraries
The flight software uses Adafruit CircuitPython modules

    pip3 install adafruit-circuitpython-bmp3xx
    pip3 install adafruit-circuitpython-gps
    pip3 install adafruit-circuitpython-bno055
    pip install adafruit-circuitpython-ina23x
    pip3 install adafruit-circuitpython-motor

## 3. Install Video Related modules

Picamera2 module is used in Raspberry Pi Camera recording
It is pre-installed on Rapsberry Pi OS images

    sudo apt install python3-picamera2
    
    sudo apt install -y libcamera-apps libcamera-tools

    sudo nano /boot/firmware/config.txt
    camera_auto_detect=0
    [all]
    dtoverlay=imx708

## 4. Install Basic modules
Other basic modules should be installed too

    pip3 install numpy==1.26.4
## 5. Install Pigpio

    pip3 install pigpio
    sudo systemctl enable pigpiod

# Run Flight software
## Update Submodules

    git submodule init
    git submodule update

## SELECT CONFIG
when initially running flight software, you will get the error

    #################################################################
    Config file does not exist: lib/config.txt, Configure the config file to run FSW!
    #################################################################

you should specify the operation mode of the FSW by editing the lib/config.txt file

# Optional Configuraton
## Granting permission to interface (Not Root)
When not running on root, permission to interfaces should be given

    sudo usermod -aG i2c {user_name}
    sudo usermod -aG gpio {user_name}
    sudo usermod -aG video {user_name}

## Optimizations
I2C clock stretching is recommended for stable use of sensors that use I2C
https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching

## When running on Raspberry Pi 5 Family (Raspberry Pi 5, Raspberry Pi Compute Module 5)
There is a bug with adafruit board library (https://forums.raspberrypi.com/viewtopic.php?t=386485) on raspberry pi 5 family, so firmware downgrade is needed until the developers fix that bug

    sudo rpi-update 0ccee17

## When using USB camera using pads
we need to configure the pi.

on /boot/firmware/cmdline.txt, add the following after rootwait

    modules-load=dwc2,g_ether

on /boot/firmware/config.txt, add the following 

    dtoverlay=dwc2,dr_mode=host 
    
on /etc/modules, add

    dwc2
