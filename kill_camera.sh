#!/bin/bash
# Force kill any processes using the camera

echo "Cleaning up camera resources..."
sudo pkill -9 libcamera
sudo pkill -9 python3
sudo rm -f /dev/shm/libcamera*

# Wait a moment for resources to be freed
sleep 1
echo "Camera resources cleanup complete."
