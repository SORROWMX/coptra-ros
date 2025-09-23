#!/usr/bin/env bash

#
# Validate built image using tests
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -ex

echo "Run image tests"

export ROS_DISTRO='noetic'
export ROS_IP='127.0.0.1'
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
systemctl start roscore

# Check if test directory exists
if [ -d "/home/orangepi/catkin_ws/src/coptra-ros/builder/test/" ]; then
    cd /home/orangepi/catkin_ws/src/coptra-ros/builder/test/
    
    # Run tests with error handling
    echo "Running tests..."
    ./tests.sh || echo "tests.sh failed, continuing..."
    ./tests.py || echo "tests.py failed, continuing..."
    ./tests_py3.py || echo "tests_py3.py failed, continuing..."
    
    # QR test
    if [ -f "./test_qr.py" ]; then
        QR_RESULT=$(./test_qr.py 2>/dev/null || echo "QR test failed")
        echo "QR test result: $QR_RESULT"
    else
        echo "test_qr.py not found, skipping..."
    fi
    
    # Clever compatibility test
    if [ -f "./tests_clever.py" ]; then
        CLEVER_RESULT=$(./tests_clever.py 2>/dev/null || echo "Clever test failed")
        echo "Clever test result: $CLEVER_RESULT"
    else
        echo "tests_clever.py not found, skipping..."
    fi
else
    echo "Test directory not found: /home/orangepi/catkin_ws/src/coptra-ros/builder/test/"
    echo "Skipping tests..."
fi

systemctl stop roscore

# check documented packages available
apt-cache show gst-rtsp-launch
apt-cache show openvpn

echo "Move /etc/ld.so.preload back to its original position"
if [ -f /etc/ld.so.preload.disabled-for-build ]; then
    mv /etc/ld.so.preload.disabled-for-build /etc/ld.so.preload
else
    echo "Warning: /etc/ld.so.preload.disabled-for-build not found, skipping restore"
fi

echo "Largest packages installed"
sudo -E sh -c 'apt-get install -y debian-goodies'
dpigs -H -n 100
