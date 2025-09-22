#! /usr/bin/env bash

#
# Script for Orange Pi 3B hardware setup
# For VM build: sudo ./build-on-vm.sh
# For Docker build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -e # Exit immidiately on non-zero result

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m$TEXT\e[0m" # BOLD

  case "$2" in
    SUCCESS)
    TEXT="\e[32m${TEXT}\e[0m";; # GREEN
    ERROR)
    TEXT="\e[31m${TEXT}\e[0m";; # RED
    *)
    TEXT="\e[34m${TEXT}\e[0m";; # BLUE
  esac
  echo -e ${TEXT}
}

##################################################
# Configure hardware interfaces
##################################################

# 1. Enable sshd
echo_stamp "#1 Turn on sshd"
touch /boot/ssh

# 2. Enable GPIO
echo_stamp "#2 GPIO enabled by default"

# 3. Enable I2C
echo_stamp "#3 Turn on I2C"
# For Orange Pi, I2C is usually enabled by default

# 4. Enable SPI
echo_stamp "#4 Turn on SPI"
# For Orange Pi, SPI is usually enabled by default

# 5. Enable camera
echo_stamp "#5 Turn on camera"
# Camera overlay is already configured in image-software.sh

# 6. Enable hardware UART
echo_stamp "#6 Turn on UART"
# For Orange Pi, UART configuration is different

# 7. Enable V4L driver
echo_stamp "#7 Turn on v4l2 driver"
# For Orange Pi with OV5647, the driver is different

echo_stamp "#8 End of configure hardware interfaces"
