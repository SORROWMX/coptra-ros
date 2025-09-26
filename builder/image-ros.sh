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

set -e # exit on error, but don't echo commands (we'll handle errors manually)

# Increase stack size to prevent segmentation faults during compilation
ulimit -s unlimited 2>/dev/null || true

# Debug: print basic system and python info
echo_stamp "DEBUG: uname -a: $(uname -a)"
echo_stamp "DEBUG: gcc --version: $(gcc --version | head -n1)"
echo_stamp "DEBUG: python3 --version: $(python3 --version 2>&1)"

REPO=$1
REF=$2
INSTALL_ROS_PACK_SOURCES=$3
DISCOVER_ROS_PACK=$4
NUMBER_THREADS=$5

# Current ROS distribution
ROS_DISTRO=noetic


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

# Debug: show key env vars
echo_stamp "DEBUG: PATH=$PATH"
echo_stamp "DEBUG: LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo_stamp "DEBUG: PYTHONPATH=$PYTHONPATH"
echo_stamp "DEBUG: CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo_stamp "DEBUG: cmake --version: $(cmake --version | head -n1 2>/dev/null || echo 'cmake not found')"

# Debug helper to inspect build state after a failure
debug_build_state() {
  local phase="$1"
  echo_stamp "DEBUG[$phase]: Dump key env vars"
  echo_stamp "DEBUG[$phase]: MAKEFLAGS=${MAKEFLAGS} CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}"
  echo_stamp "DEBUG[$phase]: GENMSG_LANGS=${GENMSG_LANGS}"
  echo_stamp "DEBUG[$phase]: CATKIN_DEVEL_PREFIX=${CATKIN_DEVEL_PREFIX}"
  echo_stamp "DEBUG[$phase]: Check generated headers under devel/include"
  ls -l /home/orangepi/catkin_ws/devel/include 2>/dev/null | head -n 50 || true
  ls -l /home/orangepi/catkin_ws/devel/include/coptra 2>/dev/null || true
  echo_stamp "DEBUG[$phase]: Search for OpticalFlowArduPilot in build tree"
  grep -R "OpticalFlowArduPilot" -n /home/orangepi/catkin_ws/build 2>/dev/null | head -n 50 || true
  echo_stamp "DEBUG[$phase]: List coptra generate_messages targets (make help)"
  (cd /home/orangepi/catkin_ws/build 2>/dev/null && make help | grep -E "coptra(_generate_messages|_gencpp)" || true)
}




safe_install "my_travis_retry apt-get install -y --no-install-recommends libboost-dev libboost-all-dev" "Installing libboost-dev"

echo_stamp "Build and install Coptra"
cd /home/orangepi/catkin_ws
# Don't try to install gazebo_ros
# ROSDEP INSTALLATION REMOVED - causing duplicate package conflicts
# safe_install "my_travis_retry rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --os=debian:bookworm --skip-keys=gazebo_ros --skip-keys=gazebo_plugins" "Install ROS dependencies"
safe_install "my_travis_retry pip3 install wheel" "Install pip wheel"
safe_install "my_travis_retry pip3 install -r /home/orangepi/catkin_ws/src/coptra-ros/coptra/requirements.txt" "Install Python requirements"
source /opt/ros/${ROS_DISTRO}/setup.bash
# Debug: after sourcing ROS
echo_stamp "DEBUG: After source /opt/ros/${ROS_DISTRO}/setup.bash"
echo_stamp "DEBUG: which catkin_make: $(command -v catkin_make || echo 'not found')"
echo_stamp "DEBUG: catkin CMake config present? $(test -f /opt/ros/${ROS_DISTRO}/share/catkin/cmake/catkinConfig.cmake && echo yes || echo no)"
echo_stamp "DEBUG: ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
chown -R orangepi:orangepi /home/orangepi/catkin_ws/
chmod -R 755 /home/orangepi/catkin_ws/

# Ensure core ROS message packages are present (std_msgs, geometry_msgs, sensor_msgs, visualization_msgs, genmsg, etc.)
echo_stamp "Ensuring ROS message packages are installed"
STD_MSGS_PATH="/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake/std_msgs-msg-paths.cmake"
GEOM_MSGS_PATH="/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake/geometry_msgs-msg-paths.cmake"
SENSOR_MSGS_PATH="/opt/ros/${ROS_DISTRO}/share/sensor_msgs/cmake/sensor_msgs-msg-paths.cmake"
VIS_MSGS_PATH="/opt/ros/${ROS_DISTRO}/share/visualization_msgs/cmake/visualization_msgs-msg-paths.cmake"
LED_MSGS_PATH="/opt/ros/${ROS_DISTRO}/share/led_msgs/cmake/led_msgs-msg-paths.cmake"
MSG_GEN_CONFIG="/opt/ros/${ROS_DISTRO}/share/message_generation/cmake/message_generationConfig.cmake"
GENMSG_CONFIG="/opt/ros/${ROS_DISTRO}/share/genmsg/cmake/genmsg-extras.cmake"
echo_stamp "DEBUG: std_msgs-msg-paths exists? $(test -f "$STD_MSGS_PATH" && echo yes || echo no)"
echo_stamp "DEBUG: geometry_msgs-msg-paths exists? $(test -f "$GEOM_MSGS_PATH" && echo yes || echo no)"
echo_stamp "DEBUG: sensor_msgs-msg-paths exists? $(test -f "$SENSOR_MSGS_PATH" && echo yes || echo no)"
echo_stamp "DEBUG: visualization_msgs-msg-paths exists? $(test -f "$VIS_MSGS_PATH" && echo yes || echo no)"
echo_stamp "DEBUG: led_msgs-msg-paths exists? $(test -f "$LED_MSGS_PATH" && echo yes || echo no)"
echo_stamp "DEBUG: message_generationConfig exists? $(test -f "$MSG_GEN_CONFIG" && echo yes || echo no)"
echo_stamp "DEBUG: genmsg-extras exists? $(test -f "$GENMSG_CONFIG" && echo yes || echo no)"
NEED_INSTALL=false
if [ ! -f "$STD_MSGS_PATH" ] || [ ! -f "$GEOM_MSGS_PATH" ] || [ ! -f "$MSG_GEN_CONFIG" ] || [ ! -f "$GENMSG_CONFIG" ] || [ ! -f "$SENSOR_MSGS_PATH" ]; then
  NEED_INSTALL=true
fi
if [ "$NEED_INSTALL" = true ]; then
  safe_install "my_travis_retry apt-get update -y" "apt update"
  safe_install "my_travis_retry apt-get install -y \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-led-msgs \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-message-runtime \
    ros-${ROS_DISTRO}-genmsg \
    ros-${ROS_DISTRO}-gencpp \
    ros-${ROS_DISTRO}-genpy" "Install ROS message packages"
  # Re-source after installing message toolchain
  source /opt/ros/${ROS_DISTRO}/setup.bash
  echo_stamp "DEBUG: Re-sourced after installing message packages"
  echo_stamp "DEBUG: Using these files now: std_msgs=$(test -f "$STD_MSGS_PATH" && echo yes || echo no), geometry_msgs=$(test -f "$GEOM_MSGS_PATH" && echo yes || echo no), sensor_msgs=$(test -f "$SENSOR_MSGS_PATH" && echo yes || echo no), visualization_msgs=$(test -f "$VIS_MSGS_PATH" && echo yes || echo no), led_msgs=$(test -f "$LED_MSGS_PATH" && echo yes || echo no), message_generation=$(test -f "$MSG_GEN_CONFIG" && echo yes || echo no), genmsg=$(test -f "$GENMSG_CONFIG" && echo yes || echo no)"
fi

# Configure catkin workspace
echo_stamp "Ensuring catkin (catkin_make) is available"
UNDERLAY="/opt/ros/${ROS_DISTRO}"
CMAKE_PREFIX_ARG="-DCMAKE_PREFIX_PATH=${UNDERLAY}"
if [ ! -x "${UNDERLAY}/bin/catkin_make" ] || [ ! -f "${UNDERLAY}/share/catkin/cmake/catkinConfig.cmake" ]; then
  safe_install "my_travis_retry apt-get update -y" "apt update"
  safe_install "my_travis_retry apt-get install -y ros-${ROS_DISTRO}-catkin" "Install ros-${ROS_DISTRO}-catkin"
  # Fallback: ensure base ROS meta-package present if catkin CMake config is still missing
  if [ ! -f "${UNDERLAY}/share/catkin/cmake/catkinConfig.cmake" ]; then
    safe_install "my_travis_retry apt-get install -y ros-${ROS_DISTRO}-ros-base" "Install ros-${ROS_DISTRO}-ros-base"
  fi
fi
# Re-source ROS environment to pick up just-installed catkin
sudo -u orangepi bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
# Debug: verify catkin again
echo_stamp "DEBUG: After ensuring catkin"
echo_stamp "DEBUG: which catkin_make: $(command -v catkin_make || echo 'not found')"
echo_stamp "DEBUG: catkin CMake config present? $(test -f /opt/ros/${ROS_DISTRO}/share/catkin/cmake/catkinConfig.cmake && echo yes || echo no)"
echo_stamp "DEBUG: ls /opt/ros/${ROS_DISTRO}/share/catkin/cmake: $(ls -1 /opt/ros/${ROS_DISTRO}/share/catkin/cmake 2>/dev/null | tr '\n' ' ')"

# Ensure CMake and ROS search paths are explicitly set for this shell
export CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH}"
export ROS_PACKAGE_PATH="/opt/ros/${ROS_DISTRO}/share:/home/orangepi/catkin_ws/src:${ROS_PACKAGE_PATH}"
echo_stamp "DEBUG: Exported CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo_stamp "DEBUG: Exported ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"

# Start from a clean build to avoid stale cache issues
rm -rf /home/orangepi/catkin_ws/build /home/orangepi/catkin_ws/devel
echo_stamp "DEBUG: Cleaned build/ and devel/"

# Pre-create devel msg-path symlinks so genmsg finds msg-paths even if it prefers devel
DEVEL_SHARE="/home/orangepi/catkin_ws/devel/share"
declare -A MSG_PKGS
MSG_PKGS=( \
  [std_msgs]="/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake/std_msgs-msg-paths.cmake" \
  [geometry_msgs]="/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake/geometry_msgs-msg-paths.cmake" \
  [sensor_msgs]="/opt/ros/${ROS_DISTRO}/share/sensor_msgs/cmake/sensor_msgs-msg-paths.cmake" \
  [visualization_msgs]="/opt/ros/${ROS_DISTRO}/share/visualization_msgs/cmake/visualization_msgs-msg-paths.cmake" \
  [led_msgs]="/opt/ros/${ROS_DISTRO}/share/led_msgs/cmake/led_msgs-msg-paths.cmake" \
)
for pkg in "${!MSG_PKGS[@]}"; do
  src_path="${MSG_PKGS[$pkg]}"
  mkdir -p "${DEVEL_SHARE}/${pkg}/cmake"
  if [ -f "${src_path}" ]; then
    ln -sf "${src_path}" "${DEVEL_SHARE}/${pkg}/cmake/${pkg}-msg-paths.cmake"
    echo_stamp "DEBUG: Linked ${pkg}-msg-paths.cmake into devel/share"
  else
    echo_stamp "DEBUG: ${pkg}-msg-paths.cmake missing at ${src_path}" "ERROR"
  fi
done

# (catkin_tools blacklist removed; using catkin_make CMake args instead)

# Build with catkin_make
echo_stamp "Building ROS packages with catkin_make"
# Add memory and build optimizations to prevent segfaults during compilation
export MAKEFLAGS="-j1"  # Single threaded build to reduce memory usage
export CMAKE_BUILD_PARALLEL_LEVEL=1  # Limit parallel compilation
# Add environment variables to help with message generation on ARM
# Note: ROS_LANG_DISABLE=eus is not valid in ROS Noetic, use CATKIN_ENABLE_TESTING=OFF instead
export CATKIN_WHITELIST_PACKAGES=""  # Clear whitelist
export CATKIN_BLACKLIST_PACKAGES="aruco_pose;roswww_static;"  # Blacklist heavy/non-critical packages
# Add compiler optimizations for ARM to reduce memory usage and prevent segfaults
export CXXFLAGS="-O0 -fno-stack-protector -fno-strict-aliasing"
export CFLAGS="-O0 -fno-stack-protector -fno-strict-aliasing"
# Clear any existing build artifacts that might cause issues
rm -rf /home/orangepi/catkin_ws/build/coptra_blocks
rm -rf /home/orangepi/catkin_ws/devel/.private/coptra_blocks
rm -rf /home/orangepi/catkin_ws/build/aruco_pose
rm -rf /home/orangepi/catkin_ws/devel/.private/aruco_pose
# Add system memory management
echo 1 > /proc/sys/vm/drop_caches 2>/dev/null || true  # Clear system caches
# Increase stack size to prevent segfaults
ulimit -s unlimited 2>/dev/null || true  # Unlimited stack size
# Try building with memory optimizations
echo_stamp "Attempting build with catkin_make"
# Temporarily disable exit on error for build process
set +e
# Debug: first configure command to be executed
echo_stamp "DEBUG: Running catkin_make with args: -j1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1 -g0' -DCATKIN_ENABLE_TESTING=OFF -DBUILD_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 ${CMAKE_PREFIX_ARG} -Dcatkin_DIR=/opt/ros/${ROS_DISTRO}/share/catkin/cmake -Dstd_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake -Dgeometry_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake -Dsensor_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/sensor_msgs/cmake -Dvisualization_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/visualization_msgs/cmake -Dled_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/led_msgs/cmake -Dmessage_generation_DIR=/opt/ros/${ROS_DISTRO}/share/message_generation/cmake -Dgenmsg_DIR=/opt/ros/${ROS_DISTRO}/share/genmsg/cmake -DCATKIN_BLACKLIST_PACKAGES='aruco_pose;roswww_static' -DCATKIN_WHITELIST_PACKAGES=''"
if ! safe_install "env CMAKE_PREFIX_PATH='${CMAKE_PREFIX_PATH}' ROS_PACKAGE_PATH='${ROS_PACKAGE_PATH}' catkin_make -j1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS='-O0 -g0' \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DBUILD_TESTING=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  ${CMAKE_PREFIX_ARG} \
  -Dcatkin_DIR=/opt/ros/${ROS_DISTRO}/share/catkin/cmake \
  -Dstd_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake \
  -Dgeometry_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake \
  -Dsensor_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/sensor_msgs/cmake \
  -Dvisualization_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/visualization_msgs/cmake \
  -Dled_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/led_msgs/cmake \
  -Dmessage_generation_DIR=/opt/ros/${ROS_DISTRO}/share/message_generation/cmake \
  -Dgenmsg_DIR=/opt/ros/${ROS_DISTRO}/share/genmsg/cmake \
  -DCATKIN_BLACKLIST_PACKAGES='aruco_pose;roswww_static' \
  -DCATKIN_WHITELIST_PACKAGES=''" "Build ROS packages with catkin_make"; then
    echo_stamp "catkin_make failed, trying selective builds" "ERROR"
    debug_build_state "main"
    # Try building only coptra_blocks (some platforms need this separately)
    echo_stamp "DEBUG: Running selective build for coptra_blocks"
    safe_install "env CMAKE_PREFIX_PATH='${CMAKE_PREFIX_PATH}' ROS_PACKAGE_PATH='${ROS_PACKAGE_PATH}' catkin_make -j1 \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS='-O0 -g0' \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DBUILD_TESTING=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      ${CMAKE_PREFIX_ARG} \
      -Dstd_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake \
      -Dgeometry_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake \
      -Dsensor_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/sensor_msgs/cmake \
      -Dvisualization_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/visualization_msgs/cmake \
      -Dled_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/led_msgs/cmake \
      -Dcatkin_DIR=/opt/ros/${ROS_DISTRO}/share/catkin/cmake \
      -Dmessage_generation_DIR=/opt/ros/${ROS_DISTRO}/share/message_generation/cmake \
      -Dgenmsg_DIR=/opt/ros/${ROS_DISTRO}/share/genmsg/cmake \
      -DCATKIN_WHITELIST_PACKAGES='coptra_blocks'" "Build coptra_blocks only" || echo_stamp "coptra_blocks build failed, continuing" "ERROR"

    # Try building only coptra
    echo_stamp "DEBUG: Running selective build for coptra"
    safe_install "env CMAKE_PREFIX_PATH='${CMAKE_PREFIX_PATH}' ROS_PACKAGE_PATH='${ROS_PACKAGE_PATH}' catkin_make -j1 \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS='-O0 -g0' \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DBUILD_TESTING=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      ${CMAKE_PREFIX_ARG} \
      -Dstd_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/std_msgs/cmake \
      -Dgeometry_msgs_DIR=/opt/ros/${ROS_DISTRO}/share/geometry_msgs/cmake \
      -Dcatkin_DIR=/opt/ros/${ROS_DISTRO}/share/catkin/cmake \
      -Dmessage_generation_DIR=/opt/ros/${ROS_DISTRO}/share/message_generation/cmake \
      -Dgenmsg_DIR=/opt/ros/${ROS_DISTRO}/share/genmsg/cmake \
      -DCATKIN_WHITELIST_PACKAGES='coptra'" "Build coptra only" || echo_stamp "coptra build failed, continuing" "ERROR"
    debug_build_state "coptra"
fi
# Re-enable exit on error after build process
set -e
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

echo_stamp "Using ROS workspace coptra package only (no legacy install)"


echo_stamp "Change permissions for catkin_ws"
chown -Rf orangepi:orangepi /home/orangepi/catkin_ws
chown -Rf orangepi:orangepi /opt/ros/noetic/lib/ 2>/dev/null || true
# Fix permissions for Python packages in devel space
chown -Rf orangepi:orangepi /home/orangepi/catkin_ws/devel/lib/python3/dist-packages/ 2>/dev/null || true
cd /home/orangepi/catkin_ws
echo_stamp "Update www"
# Update www with roswww_static from orangepi user
safe_install 'sudo -u orangepi bash -lc "
  set -e
  # Source ROS environment
  source /opt/ros/noetic/setup.bash
  if [ -f /home/orangepi/catkin_ws/devel/setup.bash ]; then
    source /home/orangepi/catkin_ws/devel/setup.bash
  fi
  
  echo \"DEBUG[www]: Using ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH\"
  echo \"DEBUG[www]: Running rosrun roswww_static update from orangepi user\"
  
  # Run roswww_static update
  rosrun roswww_static update
  
  echo \"DEBUG[www]: roswww_static update completed\"
"' "Update www via roswww_static" || {
  # Fallback: populate ~/.ros/www from package www folders
  echo_stamp "Fallback: populate /home/orangepi/.ros/www from package www folders"
  mkdir -p /home/orangepi/.ros/www/coptra /home/orangepi/.ros/www/coptra_blocks
  if [ -d "/home/orangepi/catkin_ws/src/coptra-ros/coptra/www" ]; then
    rsync -a --delete /home/orangepi/catkin_ws/src/coptra-ros/coptra/www/ /home/orangepi/.ros/www/coptra/
  fi
  if [ -d "/home/orangepi/catkin_ws/src/coptra-ros/coptra_blocks/www" ]; then
    rsync -a --delete /home/orangepi/catkin_ws/src/coptra-ros/coptra_blocks/www/ /home/orangepi/.ros/www/coptra_blocks/
  fi
  chown -R orangepi:orangepi /home/orangepi/.ros/www || true
}


echo_stamp "Setup nginx web files"
# Ensure nginx and fcgiwrap are available
if ! command -v nginx >/dev/null 2>&1; then
    echo_stamp "Warning: nginx not installed, skipping web setup" "ERROR"
else

# Copy files from roswww_static to nginx directory
if [ -d /home/orangepi/.ros/www ]; then
    # Create nginx directory
    safe_install "mkdir -p /var/www/ros" "Create nginx directory"
    
    # Remove existing files to avoid conflicts
    rm -rf /var/www/ros/*
    
    # Copy real files (not symlinks) using -L flag to follow symlinks
    safe_install "cp -rL /home/orangepi/.ros/www/coptra /var/www/ros/" "Copy coptra web files (real files)"
    safe_install "cp -rL /home/orangepi/.ros/www/coptra_blocks /var/www/ros/" "Copy coptra_blocks web files (real files)"
    
    # Set proper permissions
    chown -R www-data:www-data /var/www/ros/
    chmod -R 755 /var/www/ros/
    echo_stamp "Web files copied successfully to /var/www/ros/ (real files, not symlinks)"
    
    # Create nginx configuration for ROS
    echo_stamp "Creating nginx configuration"
    safe_install "tee /etc/nginx/sites-available/ros > /dev/null << 'EOF'
server {
    listen 80;
    server_name _;
    
    # Main page - redirect to coptra
    location = / {
        return 301 /coptra/;
    }
    
    # Redirect /coptra to /coptra/ (with trailing slash)
    location = /coptra {
        return 301 /coptra/;
    }

    # Redirect /coptra_blocks to /coptra_blocks/ (with trailing slash)  
    location = /coptra_blocks {
        return 301 /coptra_blocks/;
    }
    
    # Static files from coptra/www
    location /coptra/ {
        alias /var/www/ros/coptra/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
    
    # Static files from coptra_blocks/www
    location /coptra_blocks/ {
        alias /var/www/ros/coptra_blocks/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
    
    # For static files (CSS, JS, images)
    location ~* \\.(css|js|png|jpg|jpeg|gif|ico|svg)\$ {
        root /var/www/ros;
        expires 1y;
        add_header Cache-Control \"public, immutable\";
        add_header Access-Control-Allow-Origin \"*\";
    }

    # Add favicon
    location = /favicon.ico {
        alias /var/www/ros/coptra/coptra_icon_128.png;
        expires 1y;
    }
    
    # CGI scripts for network management
    location /cgi-bin/ {
        alias /usr/lib/cgi-bin/;
        gzip off;
        fastcgi_pass unix:/var/run/fcgiwrap.socket;
        include /etc/nginx/fastcgi_params;
        fastcgi_param SCRIPT_FILENAME /usr/lib/cgi-bin\$fastcgi_script_name;
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
    
    # Enable and start fcgiwrap for CGI support
    safe_install "systemctl enable fcgiwrap" "Enable fcgiwrap"
    safe_install "systemctl start fcgiwrap" "Start fcgiwrap"
    
    # Test nginx configuration
    safe_install "nginx -t" "Test nginx configuration"
    
    # Restart nginx
    safe_install "systemctl restart nginx" "Restart nginx"
    safe_install "systemctl enable nginx" "Enable nginx"
    
    echo_stamp "Nginx configured successfully for ROS"
else
    echo_stamp "Warning: /home/orangepi/.ros/www not found, skipping web files copy"
    # Create empty directories as fallback
    safe_install "mkdir -p /var/www/ros/coptra /var/www/ros/coptra_blocks" "Create empty web directories"
    safe_install "chown -R www-data:www-data /var/www/ros" "Set web directory ownership"
    safe_install "chmod -R 755 /var/www/ros" "Set web directory permissions"
fi
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
if [ -f "/home/orangepi/catkin_ws/src/coptra-ros/builder/assets/roscore.service" ]; then
    ln -s /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/roscore.service /lib/systemd/system/
    systemctl enable roscore
    echo_stamp "roscore.service symlink created and enabled"
else
    echo_stamp "Warning: roscore.service not found, skipping symlink creation"
fi

# Then enable coptra (depends on roscore)
if [ -f "/home/orangepi/catkin_ws/src/coptra-ros/builder/assets/coptra.service" ]; then
    ln -s /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/coptra.service /lib/systemd/system/
    systemctl enable coptra
    echo_stamp "coptra.service symlink created and enabled"
else
    echo_stamp "Warning: coptra.service not found, skipping symlink creation"
fi

# Enable network setup service (runs once on boot)
if [ -f "/home/orangepi/catkin_ws/src/coptra-ros/builder/assets/network-setup.service" ]; then
    ln -s /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/network-setup.service /lib/systemd/system/
    chmod +x /home/orangepi/catkin_ws/src/coptra-ros/builder/assets/network-setup.sh
    systemctl enable network-setup
    echo_stamp "network-setup.service symlink created and enabled"
else
    echo_stamp "Warning: network-setup.service not found, skipping symlink creation"
fi

# Udev rules removed as requested

echo_stamp "Setup ROS environment"
cat << 'EOF' >> /home/orangepi/.bashrc
# ROS Environment Setup
source /opt/ros/noetic/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1
export ROS_ROOT=/opt/ros/noetic
export ROS_DISTRO=noetic
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/home/orangepi/catkin_ws/devel/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=/home/orangepi/catkin_ws/src:/opt/ros/noetic/share:/home/orangepi/catkin_ws/devel/share:$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/home/orangepi/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export PATH=/opt/ros/noetic/bin:/home/orangepi/catkin_ws/devel/lib:$PATH
export CMAKE_PREFIX_PATH=/opt/ros/noetic:/home/orangepi/catkin_ws/devel:$CMAKE_PREFIX_PATH
EOF

# Also set up for root user
cat << 'EOF' >> /root/.bashrc
# ROS Environment Setup
source /opt/ros/noetic/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1
export ROS_ROOT=/opt/ros/noetic
export ROS_DISTRO=noetic
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/home/orangepi/catkin_ws/devel/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=/home/orangepi/catkin_ws/src:/opt/ros/noetic/share:/home/orangepi/catkin_ws/devel/share:$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/home/orangepi/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export PATH=/opt/ros/noetic/bin:/home/orangepi/catkin_ws/devel/lib:$PATH
export CMAKE_PREFIX_PATH=/opt/ros/noetic:/home/orangepi/catkin_ws/devel:$CMAKE_PREFIX_PATH
EOF

echo_stamp "Reload bashrc to apply environment variables"
# Reload bashrc for current session from orangepi user
sudo -u orangepi bash -c "source /home/orangepi/.bashrc"
source /root/.bashrc
# Ensure Python can import devel-space packages even in non-sourced shells
echo_stamp "Create Python site-path .pth for devel dist-packages"
PTH_DIR="/usr/local/lib/python3/dist-packages"
mkdir -p "$PTH_DIR"
cat > "$PTH_DIR/coptra_devel.pth" << 'EOF'
/home/orangepi/catkin_ws/devel/lib/python3/dist-packages
/opt/ros/noetic/lib/python3/dist-packages
EOF
chmod 644 "$PTH_DIR/coptra_devel.pth"
echo_stamp "Created $PTH_DIR/coptra_devel.pth"

# Symlink key ROS CLI tools into /usr/local/bin so they are available without sourcing
for bin in rosrun roslaunch rospack roscd roscore; do
  if [ -x "/opt/ros/noetic/bin/$bin" ] && [ ! -e "/usr/local/bin/$bin" ]; then
    ln -s "/opt/ros/noetic/bin/$bin" "/usr/local/bin/$bin" || true
    echo_stamp "Linked $bin to /usr/local/bin"
  fi
done

echo_stamp "Create persistent ROS symlinks script and service"
# Create comprehensive symlinks script that runs on boot
safe_install "tee /usr/local/bin/create-ros-symlinks.sh > /dev/null << 'SYMLINK_EOF'
#!/bin/bash
echo \"Creating comprehensive ROS symlinks...\"

# mavros
mkdir -p /opt/ros/noetic/share/mavros/
ln -sf /opt/ros/noetic/lib/mavros/mavros_node /opt/ros/noetic/share/mavros/mavros_node

# mavros_extras
mkdir -p /opt/ros/noetic/share/mavros_extras/
ln -sf /opt/ros/noetic/lib/mavros_extras/visualization /opt/ros/noetic/share/mavros_extras/visualization

# web_video_server
mkdir -p /opt/ros/noetic/share/web_video_server/
ln -sf /opt/ros/noetic/lib/web_video_server/web_video_server /opt/ros/noetic/share/web_video_server/web_video_server

# nodelet
mkdir -p /opt/ros/noetic/share/nodelet/
ln -sf /opt/ros/noetic/lib/nodelet/nodelet /opt/ros/noetic/share/nodelet/nodelet

# tf2_ros
mkdir -p /opt/ros/noetic/share/tf2_ros/
ln -sf /opt/ros/noetic/lib/tf2_ros/static_transform_publisher /opt/ros/noetic/share/tf2_ros/static_transform_publisher

# rosbridge_server (including rosbridge_websocket)
mkdir -p /opt/ros/noetic/share/rosbridge_server/
find /opt/ros/noetic/lib/rosbridge_server -type f -executable 2>/dev/null | while read file; do
    filename=\$(basename \"\$file\")
    ln -sf \"\$file\" \"/opt/ros/noetic/share/rosbridge_server/\$filename\"
done

# rosapi
mkdir -p /opt/ros/noetic/share/rosapi/
find /opt/ros/noetic/lib/rosapi -type f -executable 2>/dev/null | while read file; do
    filename=\$(basename \"\$file\")
    ln -sf \"\$file\" \"/opt/ros/noetic/share/rosapi/\$filename\"
done

# tf2_web_republisher
mkdir -p /opt/ros/noetic/share/tf2_web_republisher/
find /opt/ros/noetic/lib/tf2_web_republisher -type f -executable 2>/dev/null | while read file; do
    filename=\$(basename \"\$file\")
    ln -sf \"\$file\" \"/opt/ros/noetic/share/tf2_web_republisher/\$filename\"
done

echo \"ROS symlinks created successfully\"
SYMLINK_EOF" "Create ROS symlinks script"

# Make the script executable
safe_install "chmod +x /usr/local/bin/create-ros-symlinks.sh" "Make symlinks script executable"

# Create systemd service for symlinks
safe_install "tee /etc/systemd/system/ros-symlinks.service > /dev/null << 'SERVICE_EOF'
[Unit]
Description=Create ROS symlinks for rosrun compatibility
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/create-ros-symlinks.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE_EOF" "Create ROS symlinks systemd service"

# Enable and start the symlinks service
safe_install "systemctl daemon-reload" "Reload systemd daemon"
safe_install "systemctl enable ros-symlinks.service" "Enable ROS symlinks service"
safe_install "systemctl start ros-symlinks.service" "Start ROS symlinks service"

echo_stamp "ROS symlinks script and service created and enabled"

echo_stamp "Create essential ROS system symlinks"
# Create symlink for rosversion (found in /usr/bin/rosversion)
safe_install "ln -sf /usr/bin/rosversion /opt/ros/noetic/bin/rosversion" "Link rosversion to ROS bin directory"

# Create symlink for _setup_util.py (use the one generated by catkin workspace)
safe_install "ln -sf /home/orangepi/catkin_ws/devel/_setup_util.py /opt/ros/noetic/_setup_util.py" "Link _setup_util.py to ROS directory"

echo_stamp "Essential ROS system symlinks created"

# Fix nginx configuration for proper external access
echo_stamp "Fixing nginx configuration for external access"
if [ -f "/etc/nginx/sites-available/default" ]; then
    # Update root directory in default nginx config
    safe_install "sed -i 's|root /var/www/html;|root /var/www;|' /etc/nginx/sites-available/default" "Update nginx root directory"
    echo_stamp "Updated nginx root directory to /var/www"
    
    # Add coptra location blocks to default nginx config
    safe_install "tee -a /etc/nginx/sites-available/default > /dev/null << 'LOCATION_EOF'

    # Static files from coptra/www
    location /coptra/ {
        alias /var/www/ros/coptra/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
    
    # Static files from coptra_blocks/www
    location /coptra_blocks/ {
        alias /var/www/ros/coptra_blocks/;
        index index.html;
        try_files \$uri \$uri/ =404;
    }
LOCATION_EOF" "Add coptra location blocks to default nginx config"
    echo_stamp "Added coptra location blocks to default nginx config"
fi

# Ensure fcgiwrap is running for CGI support
safe_install "systemctl enable fcgiwrap" "Enable fcgiwrap service"
safe_install "systemctl start fcgiwrap" "Start fcgiwrap service"

# Test and reload nginx configuration
safe_install "nginx -t" "Test nginx configuration"
safe_install "systemctl reload nginx" "Reload nginx configuration"

echo_stamp "Nginx configuration fixed for external access"

# Debug: final env confirmation
echo_stamp "DEBUG: Final which catkin_make: $(command -v catkin_make || echo 'not found')"
echo_stamp "DEBUG: Final ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo_stamp "DEBUG: Final CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"

apt-get clean -qq > /dev/null


echo_stamp "END of ROS INSTALLATION"
