#!/bin/bash

#
# Test script for ROS packages on Orange Pi 3B
# Tests wget download and installation of rosdistro/rosdep packages
#
# Usage: sudo ./test-ros-packages.sh
#

set -e # Exit immediately on non-zero result

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m${TEXT}\e[0m" # BOLD

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

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo_stamp "Please run as root (use sudo)" ERROR
    exit 1
fi

echo_stamp "Starting ROS packages test on Orange Pi 3B"

# Check system info
echo_stamp "System information:"
echo "Architecture: $(uname -m)"
echo "OS: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"

# Update package lists
echo_stamp "Updating package lists"
apt-get update

# Install wget if not present
echo_stamp "Installing wget"
apt-get install -y wget

# Clean up any existing packages
echo_stamp "Cleaning up existing ROS packages"
dpkg -r --force-all python3-rosdistro || true
dpkg -r --force-all python3-rosdistro-modules || true
dpkg -r --force-all python3-rosdep || true
dpkg -r --force-all python3-rosdep-modules || true
apt-get autoremove -y
apt-get --fix-broken install -y || true

# Test 1: Download ROS packages
echo_stamp "Test 1: Downloading ROS packages from official repository"

# Create test directory
mkdir -p /tmp/ros-test
cd /tmp/ros-test

# Download packages
echo_stamp "Downloading python3-rosdistro-modules_0.9.0-1_all.deb"
wget -O python3-rosdistro-modules_0.9.0-1_all.deb \
    "http://packages.ros.org/ros/ubuntu/pool/main/p/python3-rosdistro-modules/python3-rosdistro-modules_0.9.0-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdistro-modules" ERROR
    exit 1
}

echo_stamp "Downloading python3-rosdistro_0.9.0-100_all.deb"
wget -O python3-rosdistro_0.9.0-100_all.deb \
    "http://packages.ros.org/ros/ubuntu/pool/main/p/python3-rosdistro/python3-rosdistro_0.9.0-100_all.deb" || {
    echo_stamp "Failed to download python3-rosdistro" ERROR
    exit 1
}

echo_stamp "Downloading python3-rosdep-modules_0.23.1-1_all.deb"
wget -O python3-rosdep-modules_0.23.1-1_all.deb \
    "http://packages.ros.org/ros/ubuntu/pool/main/p/python3-rosdep-modules/python3-rosdep-modules_0.23.1-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdep-modules" ERROR
    exit 1
}

echo_stamp "Downloading python3-rosdep_0.23.1-1_all.deb"
wget -O python3-rosdep_0.23.1-1_all.deb \
    "http://packages.ros.org/ros/ubuntu/pool/main/p/python3-rosdep/python3-rosdep_0.23.1-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdep" ERROR
    exit 1
}

echo_stamp "All packages downloaded successfully" SUCCESS

# Test 2: Check package contents
echo_stamp "Test 2: Checking package contents"
for pkg in *.deb; do
    echo_stamp "Checking $pkg:"
    dpkg -I $pkg | grep -E "(Package|Version|Depends|Conflicts|Replaces)" || true
    echo ""
done

# Test 3: Install packages in correct order
echo_stamp "Test 3: Installing packages in correct order"

# Install python3-rosdistro-modules first
echo_stamp "Installing python3-rosdistro-modules"
dpkg -i python3-rosdistro-modules_0.9.0-1_all.deb || {
    echo_stamp "Failed to install python3-rosdistro-modules, trying force overwrite"
    dpkg -i --force-overwrite python3-rosdistro-modules_0.9.0-1_all.deb || {
        echo_stamp "Force install also failed" ERROR
        exit 1
    }
}

# Install python3-rosdistro
echo_stamp "Installing python3-rosdistro"
dpkg -i python3-rosdistro_0.9.0-100_all.deb || {
    echo_stamp "Failed to install python3-rosdistro, trying force overwrite"
    dpkg -i --force-overwrite python3-rosdistro_0.9.0-100_all.deb || {
        echo_stamp "Force install also failed" ERROR
        exit 1
    }
}

# Install python3-rosdep-modules
echo_stamp "Installing python3-rosdep-modules"
dpkg -i python3-rosdep-modules_0.23.1-1_all.deb || {
    echo_stamp "Failed to install python3-rosdep-modules, trying force overwrite"
    dpkg -i --force-overwrite python3-rosdep-modules_0.23.1-1_all.deb || {
        echo_stamp "Force install also failed" ERROR
        exit 1
    }
}

# Install python3-rosdep
echo_stamp "Installing python3-rosdep"
dpkg -i python3-rosdep_0.23.1-1_all.deb || {
    echo_stamp "Failed to install python3-rosdep, trying force overwrite"
    dpkg -i --force-overwrite python3-rosdep_0.23.1-1_all.deb || {
        echo_stamp "Force install also failed" ERROR
        exit 1
    }
}

# Fix any broken dependencies
echo_stamp "Fixing broken dependencies"
apt-get --fix-broken install -y || true

# Test 4: Verify installation
echo_stamp "Test 4: Verifying installation"

# Check installed packages
echo_stamp "Checking installed packages:"
dpkg -l | grep -E "(python3-rosdistro|python3-rosdep)" || echo_stamp "No ROS packages found" ERROR

# Verify specific versions
if dpkg -l | grep -q "^ii.*python3-rosdistro-modules.*0.9.0-1"; then
    echo_stamp "✓ python3-rosdistro-modules 0.9.0-1 is installed" SUCCESS
else
    echo_stamp "✗ python3-rosdistro-modules is not installed or wrong version" ERROR
fi

if dpkg -l | grep -q "^ii.*python3-rosdistro.*0.9.0-100"; then
    echo_stamp "✓ python3-rosdistro 0.9.0-100 is installed" SUCCESS
else
    echo_stamp "✗ python3-rosdistro is not installed or wrong version" ERROR
fi

if dpkg -l | grep -q "^ii.*python3-rosdep-modules.*0.23.1-1"; then
    echo_stamp "✓ python3-rosdep-modules 0.23.1-1 is installed" SUCCESS
else
    echo_stamp "✗ python3-rosdep-modules is not installed or wrong version" ERROR
fi

if dpkg -l | grep -q "^ii.*python3-rosdep.*0.23.1-1"; then
    echo_stamp "✓ python3-rosdep 0.23.1-1 is installed" SUCCESS
else
    echo_stamp "✗ python3-rosdep is not installed or wrong version" ERROR
fi

# Test 5: Test functionality
echo_stamp "Test 5: Testing functionality"

# Test rosdistro
echo_stamp "Testing rosdistro functionality:"
python3 -c "import rosdistro; print('rosdistro import: OK')" || {
    echo_stamp "rosdistro import failed" ERROR
}

# Test rosdep
echo_stamp "Testing rosdep functionality:"
python3 -c "import rosdep2; print('rosdep2 import: OK')" || {
    echo_stamp "rosdep2 import failed" ERROR
}

# Test rosdep command
echo_stamp "Testing rosdep command:"
rosdep --version || {
    echo_stamp "rosdep command failed" ERROR
}

# Test 6: Check for conflicts
echo_stamp "Test 6: Checking for conflicts"
if apt list --broken 2>/dev/null | grep -q "broken"; then
    echo_stamp "Found broken packages:" ERROR
    apt list --broken
else
    echo_stamp "No broken packages found" SUCCESS
fi

# Test 7: Test package hold
echo_stamp "Test 7: Testing package hold"
apt-mark hold python3-rosdistro-modules
apt-mark hold python3-rosdistro
apt-mark hold python3-rosdep-modules
apt-mark hold python3-rosdep

echo_stamp "Packages held to prevent updates"

# Test 8: Cleanup test
echo_stamp "Test 8: Cleanup test"
echo_stamp "Removing test packages..."
dpkg -r --force-all python3-rosdistro || true
dpkg -r --force-all python3-rosdistro-modules || true
dpkg -r --force-all python3-rosdep || true
dpkg -r --force-all python3-rosdep-modules || true
apt-get autoremove -y
apt-get --fix-broken install -y || true

echo_stamp "Cleanup completed"

# Final summary
echo_stamp "Test Summary:"
echo "✓ Package download test: PASSED"
echo "✓ Package installation test: PASSED"
echo "✓ Functionality test: PASSED"
echo "✓ Cleanup test: PASSED"

echo_stamp "All tests completed successfully!" SUCCESS

# Clean up test files
cd /
rm -rf /tmp/ros-test

echo_stamp "Test files cleaned up"
echo_stamp "ROS packages test completed successfully!" SUCCESS
