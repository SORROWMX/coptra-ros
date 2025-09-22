#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo.
# For build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
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
SOURCE_IMAGE="https://github.com/SORROWMX/orangepi3b-ros-noetic/releases/download/debian/Orangepi3b_1.0.8_debian_bookworm_server_linux5.10.160.7z"

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
export LANG=${LANG:='C.UTF-8'}
export LC_ALL=${LC_ALL:='C.UTF-8'}

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

BUILDER_DIR="/builder"
REPO_DIR="${BUILDER_DIR}/repo"
SCRIPTS_DIR="${REPO_DIR}/builder"
IMAGES_DIR="${REPO_DIR}/images"

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
  local RPI_IMAGE_NAME=$(echo ${RPI_ARCHIVE_NAME} | sed 's/\.7z$/.img/' | sed 's/\.zip$/.img/')

  if [ ! -e "${BUILD_DIR}/${RPI_ARCHIVE_NAME}" ]; then
    echo_stamp "Downloading original Linux distribution"
    wget --progress=dot:giga -O ${BUILD_DIR}/${RPI_ARCHIVE_NAME} $2
    echo_stamp "Downloading complete" "SUCCESS" \
  else echo_stamp "Linux distribution already donwloaded"; fi

  echo_stamp "Extracting Linux distribution image"
  
  # Check file extension and extract accordingly
  if [[ ${RPI_ARCHIVE_NAME} == *.7z ]]; then
    echo_stamp "Extracting 7z archive"
    # Install 7zip if not available
    which 7z >/dev/null 2>&1 || (apt-get update && apt-get install -y p7zip-full)
    7z x ${BUILD_DIR}/${RPI_ARCHIVE_NAME} -o${BUILD_DIR}/ -y
    # Find the extracted .img file
    EXTRACTED_IMG=$(find ${BUILD_DIR}/ -name "*.img" -type f | head -1)
    if [ -n "$EXTRACTED_IMG" ]; then
      cp "$EXTRACTED_IMG" $1
      echo_stamp "7z extraction complete" "SUCCESS"
    else
      echo_stamp "No .img file found in 7z archive!" "ERROR"
      exit 1
    fi
  elif [[ ${RPI_ARCHIVE_NAME} == *.zip ]]; then
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
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-init.sh' ${IMAGE_VERSION} ${SOURCE_IMAGE}

# Copy cloned repository to the image
# Include dotfiles in globs (asterisks)
shopt -s dotglob
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
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-software.sh'
# network setup
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-network.sh'
# avahi setup
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/avahi-services/sftp-ssh.service' '/etc/avahi/services'


# Coptra (Clover for Orange Pi)
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/noetic-rosdep-coptra.yaml' '/etc/ros/rosdep/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/ros_python_paths' '/etc/sudoers.d/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/pigpiod.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/launch.nanorc' '/usr/share/nano/'
# Add rename script
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-ros.sh' ${REPO_URL} ${IMAGE_VERSION} false false ${NUMBER_THREADS}
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-validate.sh'

${BUILDER_DIR}/image-resize.sh ${IMAGE_PATH}
