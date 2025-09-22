#! /usr/bin/env bash

#
# Script for building Orange Pi 3B Coptra image
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

# Orange Pi 3B Debian Bookworm image with ROS Noetic
SOURCE_IMAGE="https://github.com/SORROWMX/orangepi3b-ros-noetic/releases/download/debian/Orangepi3b_1.0.8_debian_bookworm_server_linux5.10.160.zip"

# Alternative download URLs (fallback)
ALTERNATIVE_URLS=(
  "https://github.com/SORROWMX/orangepi3b-ros-noetic/releases/download/debian/Orangepi3b_1.0.8_debian_bookworm_server_linux5.10.160.zip"
  "https://github.com/SORROWMX/orangepi3b-ros-noetic/releases/latest/download/Orangepi3b_1.0.8_debian_bookworm_server_linux5.10.160.zip"
)

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
export LANG=${LANG:='C.UTF-8'}
export LC_ALL=${LC_ALL:='C.UTF-8'}

# Force rebuild option
FORCE_REBUILD=${FORCE_REBUILD:-false}

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

# Detect if running in Docker or on VM
if [ -d "/builder" ]; then
    # Running in Docker
    BUILDER_DIR="/builder"
    REPO_DIR="${BUILDER_DIR}/repo"
    SCRIPTS_DIR="${REPO_DIR}/builder"
    IMAGES_DIR="${REPO_DIR}/images"
    echo_stamp "Running in Docker environment"
else
    # Running on VM
    BUILDER_DIR="$(dirname "$(readlink -f "$0")")"
    REPO_DIR="$(dirname "$BUILDER_DIR")"
    SCRIPTS_DIR="$BUILDER_DIR"
    IMAGES_DIR="${REPO_DIR}/images"
    echo_stamp "Running on VM environment"
fi

echo_stamp "Paths:"
echo_stamp "  BUILDER_DIR: $BUILDER_DIR"
echo_stamp "  REPO_DIR: $REPO_DIR"
echo_stamp "  SCRIPTS_DIR: $SCRIPTS_DIR"
echo_stamp "  IMAGES_DIR: $IMAGES_DIR"

[[ ! -d ${SCRIPTS_DIR} ]] && (echo_stamp "Directory ${SCRIPTS_DIR} doesn't exist" "ERROR"; exit 1)
[[ ! -d ${IMAGES_DIR} ]] && mkdir ${IMAGES_DIR} && echo_stamp "Directory ${IMAGES_DIR} was created successful" "SUCCESS"

if [[ -z ${TRAVIS_TAG} ]]; then IMAGE_VERSION="$(cd ${REPO_DIR}; git log --format=%h -1)"; else IMAGE_VERSION="${TRAVIS_TAG}"; fi
# IMAGE_VERSION="${TRAVIS_TAG:=$(cd ${REPO_DIR}; git log --format=%h -1)}"
REPO_URL="$(cd ${REPO_DIR}; git remote --verbose | grep origin | grep fetch | cut -f2 | cut -d' ' -f1 | sed 's/git@github\.com\:/https\:\/\/github.com\//')"
REPO_NAME="$(basename -s '.git' ${REPO_URL})"
IMAGE_NAME="${REPO_NAME}_${IMAGE_VERSION}.img"
IMAGE_PATH="${IMAGES_DIR}/${IMAGE_NAME}"

get_image() {
  # TEMPLATE: get_image <IMAGE_PATH> <RPI_DONWLOAD_URL>
  local BUILD_DIR=$(dirname $1)
  local RPI_ARCHIVE_NAME=$(basename $2)
  local RPI_IMAGE_NAME=$(echo ${RPI_ARCHIVE_NAME} | sed 's/\.zip$/.img/')

  # Check if final image already exists
  if [ -f "$1" ] && [ "$FORCE_REBUILD" != "true" ]; then
    echo_stamp "Image already exists: $1" "SUCCESS"
    echo_stamp "Skipping download and extraction"
    echo_stamp "Use FORCE_REBUILD=true to force rebuild"
    return 0
  elif [ -f "$1" ] && [ "$FORCE_REBUILD" = "true" ]; then
    echo_stamp "Force rebuild enabled, removing existing image: $1"
    rm -f "$1"
  fi

  # Check if archive exists
  if [ ! -e "${BUILD_DIR}/${RPI_ARCHIVE_NAME}" ]; then
    echo_stamp "Downloading original Linux distribution"
    
    # Try multiple URLs and methods
    DOWNLOAD_SUCCESS=false
    for url in "${ALTERNATIVE_URLS[@]}"; do
      echo_stamp "Trying URL: $url"
      
      # Try wget with SSL options
      if wget --progress=dot:giga --no-check-certificate --timeout=30 --tries=3 -O ${BUILD_DIR}/${RPI_ARCHIVE_NAME} "$url"; then
        DOWNLOAD_SUCCESS=true
        break
      fi
      
      echo_stamp "wget failed for $url, trying curl"
      # Fallback to curl
      if curl -L --insecure --connect-timeout 30 --max-time 300 -o ${BUILD_DIR}/${RPI_ARCHIVE_NAME} "$url"; then
        DOWNLOAD_SUCCESS=true
        break
      fi
      
      echo_stamp "Both wget and curl failed for $url"
    done
    
    if [ "$DOWNLOAD_SUCCESS" = false ]; then
      echo_stamp "All download methods failed!" "ERROR"
      exit 1
    fi
    
    echo_stamp "Downloading complete" "SUCCESS"
  else 
    echo_stamp "Archive already downloaded: ${BUILD_DIR}/${RPI_ARCHIVE_NAME}"
  fi

  echo_stamp "Extracting Linux distribution image"
  
  # Extract ZIP archive
  if [[ ${RPI_ARCHIVE_NAME} == *.zip ]]; then
    echo_stamp "Extracting zip archive"
    unzip -p ${BUILD_DIR}/${RPI_ARCHIVE_NAME} ${RPI_IMAGE_NAME} > $1
    echo_stamp "Zip extraction complete" "SUCCESS"
  else
    echo_stamp "Unsupported archive format: ${RPI_ARCHIVE_NAME}" "ERROR"
    exit 1
  fi
}

get_image ${IMAGE_PATH} ${SOURCE_IMAGE}


${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/init_rpi.sh' '/root/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/hardware_setup.sh' '/root/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec "${BUILDER_DIR}/image-init.sh" ${IMAGE_VERSION} ${SOURCE_IMAGE}

# Copy cloned repository to the image
# Include dotfiles in globs (asterisks)
shopt -s dotglob

# Create target directory structure first
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec '/bin/bash' -c 'mkdir -p /home/orangepi/catkin_ws/src/coptra'

for dir in ${REPO_DIR}/*; do
  # Don't try to copy image into itself
  if [[ $dir != *"images" && $dir != *"imgcache" ]]; then
    ${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy $dir '/home/orangepi/catkin_ws/src/coptra/'
  fi;
done


# rsyslog config
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/rsyslog.conf' '/etc'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/rsysrot.sh' '/etc/rsyslog.d'
# Butterfly
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/butterfly.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/butterfly.socket' '/lib/systemd/system/'
# software install
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec "${BUILDER_DIR}/image-software.sh"
# network setup
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec "${BUILDER_DIR}/image-network.sh"
# avahi setup
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/avahi-services/sftp-ssh.service' '/etc/avahi/services'


# Coptra (Clover for Orange Pi)
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/noetic-rosdep-coptra.yaml' '/etc/ros/rosdep/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/ros_python_paths' '/etc/sudoers.d/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/pigpiod.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/launch.nanorc' '/usr/share/nano/'
# Add rename script
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec "${BUILDER_DIR}/image-ros.sh" ${REPO_URL} ${IMAGE_VERSION} false false ${NUMBER_THREADS}
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec "${BUILDER_DIR}/image-validate.sh"

${BUILDER_DIR}/image-resize.sh ${IMAGE_PATH}
