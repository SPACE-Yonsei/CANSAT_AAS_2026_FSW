#!/bin/bash
# Install missing dependencies in the virtual environment

echo "Activating virtual environment..."
source ~/env/bin/activate

echo "Installing required libraries..."
pip3 install --upgrade adafruit-python-shell
pip3 install --upgrade adafruit-blinka
pip3 install adafruit-circuitpython-bmp3xx
pip3 install adafruit-circuitpython-bno08x
pip3 install adafruit-circuitpython-motor
pip3 install adafruit-circuitpython-ina23x
pip3 install adafruit-circuitpython-vl53l1x
pip3 install pigpio
pip3 install smbus2
pip3 install opencv-python
pip3 install numpy==1.26.4

echo "Installation complete."
