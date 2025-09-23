#!/bin/bash
# Install dependencies for Coptra packages

set -e

echo "Installing Coptra dependencies..."

# Update package lists
sudo apt-get update

# Install ROS Noetic dependencies
echo "Installing ROS Noetic dependencies..."

# Check if python3-rospy is available in repositories
if apt-cache policy python3-rospy | grep -q "Candidate:" && ! apt-cache policy python3-rospy | grep -q "Candidate: (none)"; then
    echo "python3-rospy found in repositories, installing normally"
    sudo apt-get install -y python3-rospy
else
    echo "python3-rospy not found in repositories, downloading from Debian archive"
    wget -O /tmp/python3-rospy_1.15.15+ds-2_all.deb http://ftp.us.debian.org/debian/pool/main/r/ros-ros-comm/python3-rospy_1.15.15+ds-2_all.deb
    sudo dpkg -i /tmp/python3-rospy_1.15.15+ds-2_all.deb
    sudo apt-get --fix-broken install -y
    rm -f /tmp/python3-rospy_1.15.15+ds-2_all.deb
fi

sudo apt-get install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-message-generation \
  ros-noetic-message-runtime \
  ros-noetic-nodelet \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-std-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-visualization-msgs \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-image-proc \
  ros-noetic-image-geometry \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  ros-noetic-gscam \
  ros-noetic-rosbridge-server \
  ros-noetic-web-video-server \
  ros-noetic-tf2-web-republisher \
  ros-noetic-topic-tools

# Install Python dependencies
echo "Installing Python dependencies..."
sudo apt-get install -y python3-pymavlink python3-yaml python3-lxml

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get install -y \
  libxml2-dev \
  libxslt1-dev \
  libopencv-dev \
  python3-opencv \
  python3-pip

# Install Orange Pi ROS repository
echo "Setting up Orange Pi ROS repository..."
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://sorrowmx.github.io/orangepi3b-ros-noetic/public-key.asc | sudo gpg --dearmor -o /etc/apt/keyrings/orangepi-ros-noetic.gpg
sudo chmod 0644 /etc/apt/keyrings/orangepi-ros-noetic.gpg

echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/orangepi-ros-noetic.gpg] https://sorrowmx.github.io/orangepi3b-ros-noetic/debian bookworm main" | sudo tee /etc/apt/sources.list.d/ros-noetic-orangepi.list
sudo apt update

echo "Dependencies installation complete!"
