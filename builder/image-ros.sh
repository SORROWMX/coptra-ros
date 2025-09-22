#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo
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

set -ex # exit on error, echo commands

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

# TODO: 'noetic-rosdep-coptra.yaml' should add only if we use our repo?
echo_stamp "Init rosdep"
my_travis_retry rosdep init
# FIXME: Re-add this after missing packages are built
echo "yaml file:///etc/ros/rosdep/${ROS_DISTRO}-rosdep-coptra.yaml" >> /etc/ros/rosdep/sources.list.d/10-coptra.list
my_travis_retry rosdep update --include-eol-distros

echo_stamp "Populate rosdep for ROS user"
my_travis_retry sudo -u orangepi rosdep update --include-eol-distros

export ROS_IP='127.0.0.1' # needed for running tests

# echo_stamp "Reconfiguring Coptra repository for simplier unshallowing"
cd /home/orangepi/catkin_ws/src/coptra
git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*"

echo_stamp "Installing additional ROS packages for Orange Pi"
apt install -y --no-install-recommends \
ros-${ROS_DISTRO}-compressed-image-transport \
ros-${ROS_DISTRO}-cv-bridge \
ros-${ROS_DISTRO}-cv-camera \
ros-${ROS_DISTRO}-image-publisher \
ros-${ROS_DISTRO}-web-video-server

echo_stamp "Installing libboost-dev" # https://travis-ci.org/github/CopterExpress/clover/jobs/766318908#L6536
my_travis_retry apt-get install -y --no-install-recommends libboost-dev libboost-all-dev

echo_stamp "Build and install Coptra"
cd /home/orangepi/catkin_ws
# Don't try to install gazebo_ros
my_travis_retry rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --os=debian:bookworm \
  --skip-keys=gazebo_ros --skip-keys=gazebo_plugins
my_travis_retry pip3 install wheel
my_travis_retry pip3 install -r /home/orangepi/catkin_ws/src/coptra/coptra/requirements.txt
source /opt/ros/${ROS_DISTRO}/setup.bash
# Don't build simulation plugins for actual drone
catkin_make -j2 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash

echo_stamp "Install clever package (for backwards compatibility)"
cd /home/orangepi/catkin_ws/src/coptra/builder/assets/clever
./setup.py install
rm -rf build  # remove build artifacts

cd /home/orangepi/catkin_ws/src/coptra
echo_stamp "Installing additional ROS packages"
my_travis_retry apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-rosserial \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-vl53l1x \
    ros-${ROS_DISTRO}-ws281x \
    ros-${ROS_DISTRO}-rosshow \
    ros-${ROS_DISTRO}-cmake-modules \
    ros-${ROS_DISTRO}-image-view \
    ros-${ROS_DISTRO}-nodelet-topic-tools \
    ros-${ROS_DISTRO}-stereo-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-angles

# TODO move GeographicLib datasets to Mavros debian package
echo_stamp "Install GeographicLib datasets (needed for mavros)" \
&& wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

echo_stamp "Running tests"
cd /home/orangepi/catkin_ws
# FIXME: Investigate failing tests
catkin_make run_tests #&& catkin_test_results

echo_stamp "Change permissions for catkin_ws"
chown -Rf orangepi:orangepi /home/orangepi/catkin_ws

echo_stamp "Update www"
sudo -u orangepi sh -c ". devel/setup.sh && rosrun roswww_static update"

echo_stamp "Setup nginx web files"
# Copy files from roswww_static to nginx directory
cp -r /home/orangepi/.ros/www/* /var/www/ros/
chown -R www-data:www-data /var/www/ros/
chmod -R 755 /var/www/ros/

echo_stamp "Make \$HOME/examples symlink"
ln -s "$(catkin_find coptra examples --first-only)" /home/orangepi
chown -Rf orangepi:orangepi /home/orangepi/examples

echo_stamp "Make systemd services symlinks"
ln -s /home/orangepi/catkin_ws/src/coptra/builder/assets/coptra.service /lib/systemd/system/
ln -s /home/orangepi/catkin_ws/src/coptra/builder/assets/roscore.service /lib/systemd/system/
# validate
[ -f /lib/systemd/system/coptra.service ]
[ -f /lib/systemd/system/roscore.service ]

echo_stamp "Make udev rules symlink"
ln -s "$(catkin_find coptra udev --first-only)"/* /lib/udev/rules.d/

echo_stamp "Setup ROS environment"
cat << EOF >> /home/orangepi/.bashrc
LANG='C.UTF-8'
LC_ALL='C.UTF-8'
export ROS_HOSTNAME=\`hostname\`.local
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/orangepi/catkin_ws/devel/setup.bash
EOF

#echo_stamp "Removing local apt mirror"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean -qq > /dev/null
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo_stamp "END of ROS INSTALLATION"
