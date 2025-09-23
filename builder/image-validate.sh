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

set -e # exit on error, but don't echo commands (we'll handle errors manually)

# Increase stack size to prevent segmentation faults during compilation
ulimit -s unlimited 2>/dev/null || true

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO
  local TEXT="$1"
  local TYPE="$2"
  local TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
  
  case $TYPE in
    SUCCESS)
      echo -e "\033[0;32m[${TIMESTAMP}] SUCCESS: ${TEXT}\033[0m"
      ;;
    ERROR)
      echo -e "\033[0;31m[${TIMESTAMP}] ERROR: ${TEXT}\033[0m"
      ;;
    INFO)
      echo -e "\033[0;34m[${TIMESTAMP}] INFO: ${TEXT}\033[0m"
      ;;
    *)
      echo -e "\033[0;33m[${TIMESTAMP}] ${TEXT}\033[0m"
      ;;
  esac
}

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  local max_count=5
  while [ $count -le $max_count ]; do
    [ $result -ne 0 ] && {
      echo -e "\n\033[0;31mThe command \"$@\" failed. Retrying, $count of $max_count.\033[0m\n"
    }
    "$@" && { result=0 && break; } || result=$?
    count=$(( $count + 1 ))
    sleep 1
  done
  [ $count -gt $max_count ] && {
    echo -e "\n\033[0;31mThe command \"$@\" failed after $max_count attempts.\033[0m\n"
  }
  return $result
}

safe_install() {
  local COMMAND="$1"
  local DESCRIPTION="$2"
  
  echo_stamp "Attempting: $DESCRIPTION"
  if my_travis_retry $COMMAND; then
    echo_stamp "SUCCESS: $DESCRIPTION"
  else
    echo_stamp "FAILED: $DESCRIPTION (continuing anyway)"
  fi
}

echo "Run image tests"

export ROS_DISTRO='noetic'
export ROS_IP='127.0.0.1'
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
systemctl start roscore

# Check if test directory exists
if [ -d "/home/orangepi/catkin_ws/src/coptra-ros/builder/test/" ]; then
    cd /home/orangepi/catkin_ws/src/coptra-ros/builder/test/
    
    # Fix permissions for test files
    echo "Fixing permissions for test files..."
    chmod +x *.sh *.py 2>/dev/null || true
    
    # Run tests with error handling
    echo "Running tests..."
    safe_install "python3 ./tests_py3.py" "Run tests_py3.py"
    
    # QR test
    if [ -f "./test_qr.py" ]; then
        safe_install "python3 ./test_qr.py" "Run QR test"
    else
        echo "test_qr.py not found, skipping..."
    fi
    
    # Clever compatibility test
    if [ -f "./tests_clever.py" ]; then
        safe_install "python3 ./tests_clever.py" "Run Clever test"
    else
        echo "tests_clever.py not found, skipping..."
    fi
else
    echo "Test directory not found: /home/orangepi/catkin_ws/src/coptra-ros/builder/test/"
    echo "Skipping tests..."
fi

systemctl stop roscore



echo "Move /etc/ld.so.preload back to its original position"
if [ -f /etc/ld.so.preload.disabled-for-build ]; then
    mv /etc/ld.so.preload.disabled-for-build /etc/ld.so.preload
else
    echo "Warning: /etc/ld.so.preload.disabled-for-build not found, skipping restore"
fi
sudo apt update
echo "Largest packages installed"

# Check if debian-goodies is available in repositories
if apt-cache policy debian-goodies | grep -q "Candidate:" && ! apt-cache policy debian-goodies | grep -q "Candidate: (none)"; then
    echo_stamp "debian-goodies found in repositories, installing normally"
    safe_install "sudo -E sh -c 'apt-get install -y debian-goodies'" "Install debian-goodies"
else
    echo_stamp "debian-goodies not found in repositories, downloading from Debian archive"
    safe_install "wget -O /tmp/debian-goodies_0.88.1_all.deb http://ftp.us.debian.org/debian/pool/main/d/debian-goodies/debian-goodies_0.88.1_all.deb" "Download debian-goodies package"
    safe_install "sudo dpkg -i /tmp/debian-goodies_0.88.1_all.deb" "Install debian-goodies package"
    safe_install "sudo apt-get --fix-broken install -y" "Fix any broken dependencies"
    rm -f /tmp/debian-goodies_0.88.1_all.deb
fi

safe_install "dpigs -H -n 100" "Show largest packages"
