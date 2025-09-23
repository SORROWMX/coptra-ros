#! /usr/bin/env bash

#
# Script for ROS setup in Orange Pi 3B Coptra image
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

set -e # exit on error, but don't echo commands (we'll handle errors manually)

REPO=$1
REF=$2
INSTALL_ROS_PACK_SOURCES=$3
DISCOVER_ROS_PACK=$4
NUMBER_THREADS=$5

# Current ROS distribution
ROS_DISTRO=noetic

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

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  local max_count=5
  while [ $count -le $max_count ]; do
    [ $result -ne 0 ] && {
      echo -e "\nThe command \"$@\" failed. Retrying, $count of $max_count.\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt $max_count ] && {
    echo -e "\nThe command \"$@\" failed $max_count times.\n" >&2
  }

  return $result
}

# Function to handle installation failures gracefully
safe_install() {
  local cmd="$1"
  local description="$2"
  
  echo_stamp "Attempting: $description"
  if eval "$cmd"; then
    echo_stamp "SUCCESS: $description" "SUCCESS"
  else
    echo_stamp "FAILED: $description (continuing anyway)" "ERROR"
    return 1
  fi
}

# TODO: 'noetic-rosdep-coptra.yaml' should add only if we use our repo?
# ROSDEP REMOVED - causing duplicate package conflicts
# safe_install "my_travis_retry rosdep init" "Init rosdep"
# FIXME: Re-add this after missing packages are built
# echo "yaml file:///etc/ros/rosdep/${ROS_DISTRO}-rosdep-coptra.yaml" >> /etc/ros/rosdep/sources.list.d/10-coptra.list
# safe_install "my_travis_retry rosdep update --include-eol-distros" "Update rosdep"



export ROS_IP='127.0.0.1' # needed for running tests




safe_install "my_travis_retry apt-get install -y --no-install-recommends libboost-dev libboost-all-dev" "Installing libboost-dev"

echo_stamp "Build and install Coptra"
cd /home/orangepi/catkin_ws
# Don't try to install gazebo_ros
# ROSDEP INSTALLATION REMOVED - causing duplicate package conflicts
# safe_install "my_travis_retry rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --os=debian:bookworm --skip-keys=gazebo_ros --skip-keys=gazebo_plugins" "Install ROS dependencies"
safe_install "my_travis_retry pip3 install wheel" "Install pip wheel"
safe_install "my_travis_retry pip3 install -r /home/orangepi/catkin_ws/src/coptra-ros/coptra/requirements.txt" "Install Python requirements"
source /opt/ros/${ROS_DISTRO}/setup.bash

# Configure catkin workspace
echo_stamp "Configuring catkin workspace"
catkin config --install --install-space /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DBUILD_TESTING=OFF

# Build with catkin build
echo_stamp "Building ROS packages with catkin build"
# Add memory and build optimizations to prevent segfaults
export MAKEFLAGS="-j1"  # Single threaded build to reduce memory usage
export CMAKE_BUILD_PARALLEL_LEVEL=1  # Limit parallel compilation
# Add environment variables to help with message generation on ARM
export ROS_LANG_DISABLE="eus"  # Disable EusLisp if causing issues
export CATKIN_WHITELIST_PACKAGES=""  # Clear whitelist
export CATKIN_BLACKLIST_PACKAGES=""  # Clear blacklist
# Add compiler optimizations for ARM to reduce memory usage
export CXXFLAGS="-O1 -g0 -fno-omit-frame-pointer -fno-stack-protector"
export CFLAGS="-O1 -g0 -fno-omit-frame-pointer -fno-stack-protector"
# Clear any existing build artifacts that might cause issues
rm -rf /home/orangepi/catkin_ws/build/coptra_blocks
rm -rf /home/orangepi/catkin_ws/devel/.private/coptra_blocks
rm -rf /home/orangepi/catkin_ws/build/aruco_pose
rm -rf /home/orangepi/catkin_ws/devel/.private/aruco_pose
# Add system memory management
echo 1 > /proc/sys/vm/drop_caches 2>/dev/null || true  # Clear system caches
# Set memory limits to prevent segfaults
ulimit -v 2097152 2>/dev/null || true  # Limit virtual memory to 2GB
# Try building with memory optimizations and disabled EusLisp
if ! safe_install "catkin build --jobs 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O2 -g0' -DROS_LANG_DISABLE=eus" "Build ROS packages"; then
    echo_stamp "First build attempt failed, trying with coptra_blocks excluded" "ERROR"
    # Try building without coptra_blocks to avoid segfault
    echo_stamp "Trying to build packages excluding coptra_blocks"
    safe_install "catkin build --jobs 1 --blacklist coptra_blocks aruco_pose --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O2 -g0' -DROS_LANG_DISABLE=eus" "Build packages excluding problematic packages"
    
    # If that works, try building coptra_blocks separately with more aggressive memory limits
    if [ $? -eq 0 ]; then
        echo_stamp "Core packages built successfully, trying coptra_blocks with memory limits"
        # Set even more restrictive memory limits for coptra_blocks
        ulimit -v 1048576 2>/dev/null || true  # Limit to 1GB
        ulimit -m 1048576 2>/dev/null || true  # Limit physical memory
        # Clear caches before building coptra_blocks
        echo 1 > /proc/sys/vm/drop_caches 2>/dev/null || true
        
        # Try building coptra_blocks with single thread and no parallel jobs
        if ! safe_install "catkin build --jobs 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1 -g0' -DROS_LANG_DISABLE=eus coptra_blocks" "Build coptra_blocks with memory limits"; then
            echo_stamp "catkin build failed, trying catkin_make_isolated for coptra_blocks" "ERROR"
            # Fallback to catkin_make_isolated which is more stable on ARM
            safe_install "catkin_make_isolated --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1 -g0' -DROS_LANG_DISABLE=eus --pkg coptra_blocks" "Build coptra_blocks with catkin_make_isolated"
        fi
        
        # Try building aruco_pose with memory limits
        echo_stamp "Trying to build aruco_pose with memory limits"
        ulimit -v 1048576 2>/dev/null || true  # Limit to 1GB
        ulimit -m 1048576 2>/dev/null || true  # Limit physical memory
        echo 1 > /proc/sys/vm/drop_caches 2>/dev/null || true
        if ! safe_install "catkin build --jobs 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1 -g0' -DROS_LANG_DISABLE=eus aruco_pose" "Build aruco_pose with memory limits"; then
            echo_stamp "catkin build failed, trying catkin_make_isolated for aruco_pose" "ERROR"
            # Fallback to catkin_make_isolated for aruco_pose
            safe_install "catkin_make_isolated --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1 -g0' -DROS_LANG_DISABLE=eus --pkg aruco_pose" "Build aruco_pose with catkin_make_isolated"
        fi
    fi
fi
# Source setup.bash from appropriate location
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo_stamp "Sourced devel/setup.bash"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo_stamp "Sourced /opt/ros/noetic/setup.bash"
else
    echo_stamp "Warning: No setup.bash found, using system ROS" "ERROR"
fi

echo_stamp "Install coptra package (for backwards compatibility)"
cd /home/orangepi/catkin_ws/src/coptra-ros/builder/assets
# Fix permissions and install with sudo
chmod +x setup.py
safe_install "sudo ./setup.py install" "Install coptra package"
rm -rf build  # remove build artifacts


echo_stamp "Change permissions for catkin_ws"
chown -Rf orangepi:orangepi /home/orangepi/catkin_ws
cd /home/orangepi/catkin_ws
echo_stamp "Update www"
# Update www with roswww_static
safe_install "sudo -u orangepi bash -c 'source /opt/ros/noetic/setup.bash && rosrun roswww_static update'" "Update www"


echo_stamp "Setup nginx web files"
# Copy files from roswww_static to nginx directory
if [ -d /home/orangepi/.ros/www ]; then
    # Create nginx directory
    safe_install "mkdir -p /var/www/ros" "Create nginx directory"
    
    # Remove existing files to avoid conflicts
    rm -rf /var/www/ros/*
    
    # Copy new files
    cp -r /home/orangepi/.ros/www/* /var/www/ros/
    chown -R www-data:www-data /var/www/ros/
    chmod -R 755 /var/www/ros/
    echo_stamp "Web files copied successfully to /var/www/ros/"
    
    # Create nginx configuration for ROS
    echo_stamp "Creating nginx configuration"
    safe_install "tee /etc/nginx/sites-available/ros > /dev/null << 'EOF'
server {
    listen 80;
    server_name localhost;
    
    # Главная страница - перенаправляем на coptra
    location = / {
        return 301 /coptra/;
    }
    
    # Статические файлы из coptra/www
    location /coptra/ {
        alias /var/www/ros/coptra/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
    
    # Статические файлы из coptra_blocks/www
    location /coptra_blocks/ {
        alias /var/www/ros/coptra_blocks/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
    
    # Для статических файлов (CSS, JS, изображения)
    location ~* \\.(css|js|png|jpg|jpeg|gif|ico|svg)\$ {
        root /var/www/ros;
        expires 1y;
        add_header Cache-Control \"public, immutable\";
    }
}
EOF" "Create nginx ROS configuration"
    
    # Create symbolic link (check if it exists first)
    if [ ! -L /etc/nginx/sites-enabled/ros ]; then
        safe_install "ln -s /etc/nginx/sites-available/ros /etc/nginx/sites-enabled/" "Enable ROS site"
    else
        echo_stamp "ROS site already enabled"
    fi
    
    # Remove default configuration (check if it exists first)
    if [ -L /etc/nginx/sites-enabled/default ]; then
        safe_install "rm -f /etc/nginx/sites-enabled/default" "Remove default nginx site"
    else
        echo_stamp "Default nginx site already removed"
    fi
    
    # Test nginx configuration
    safe_install "nginx -t" "Test nginx configuration"
    
    # Restart nginx
    safe_install "systemctl restart nginx" "Restart nginx"
    safe_install "systemctl enable nginx" "Enable nginx"
    
    echo_stamp "Nginx configured successfully for ROS"
else
    echo_stamp "Warning: /home/orangepi/.ros/www not found, skipping web files copy"
fi

echo_stamp "Make \$HOME/examples symlink"
# Check for examples in the correct path
if [ -d "/home/orangepi/catkin_ws/src/coptra-ros/coptra/examples" ]; then
    EXAMPLES_PATH="/home/orangepi/catkin_ws/src/coptra-ros/coptra/examples"
    ln -s "$EXAMPLES_PATH" /home/orangepi/examples
    chown -Rf orangepi:orangepi /home/orangepi/examples
    echo_stamp "Examples symlink created: $EXAMPLES_PATH -> /home/orangepi/examples"
else
    echo_stamp "Warning: coptra examples not found at /home/orangepi/catkin_ws/src/coptra-ros/coptra/examples, skipping symlink creation"
fi

echo_stamp "Make systemd services symlinks"
if [ -f "/home/orangepi/catkin_ws/src/coptra-ros/builder/assets/coptra.service" ]; then
    ln -s /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/coptra.service /lib/systemd/system/
    echo_stamp "coptra.service symlink created"
else
    echo_stamp "Warning: coptra.service not found, skipping symlink creation"
fi

if [ -f "/home/orangepi/catkin_ws/src/coptra-ros/builder/assets/roscore.service" ]; then
    ln -s /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/roscore.service /lib/systemd/system/
    echo_stamp "roscore.service symlink created"
else
    echo_stamp "Warning: roscore.service not found, skipping symlink creation"
fi

# Udev rules removed as requested

echo_stamp "Setup ROS environment"
cat << EOF >> /home/orangepi/.bashrc
LANG='C.UTF-8'
LC_ALL='C.UTF-8'
export ROS_HOSTNAME=$(hostname).local
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
EOF


apt-get clean -qq > /dev/null


echo_stamp "END of ROS INSTALLATION"
