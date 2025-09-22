#!/bin/bash

# Script to create necessary directories in the chroot environment
# This script is executed inside the chroot

set -e

echo "Creating target directory structure..."

# Create catkin workspace directories
mkdir -p /home/orangepi/catkin_ws/src/coptra

# Set proper ownership
chown -R orangepi:orangepi /home/orangepi/catkin_ws 2>/dev/null || true

echo "Directory structure created successfully"
