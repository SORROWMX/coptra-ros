#! /usr/bin/env bash

#
# Script for installing software to Orange Pi 3B Coptra image
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

# Increase stack size to prevent segmentation faults during compilation
ulimit -s unlimited 2>/dev/null || true

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

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

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  while [ $count -le 3 ]; do
    [ $result -ne 0 ] && {
      echo -e "\n${ANSI_RED}The command \"$@\" failed. Retrying, $count of 3.${ANSI_RESET}\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt 3 ] && {
    echo -e "\n${ANSI_RED}The command \"$@\" failed 3 times.${ANSI_RESET}\n" >&2
  }

  return $result
}

echo_stamp "Increase apt retries"

echo "APT::Acquire::Retries \"3\";" > /etc/apt/apt.conf.d/80-retries

echo_stamp "Installing dhcpcd5 for network management"
# Try to install dhcpcd5, fallback to dhcpcd or download from sorrowmx repo
my_travis_retry apt-get install -y dhcpcd5 || {
    echo_stamp "dhcpcd5 not found, trying dhcpcd"
    my_travis_retry apt-get install -y dhcpcd || {
        echo_stamp "dhcpcd not found, downloading from sorrowmx repository"
        wget -O /tmp/dhcpcd5_9.4.1-24~deb12u4_all.deb \
            "https://sorrowmx.github.io/orangepi3b-ros-noetic/debian/pool/main/arm64/dhcpcd5_9.4.1-24~deb12u4_all.deb" || {
            echo_stamp "Failed to download dhcpcd5 from sorrowmx repo" ERROR
            exit 1
        }
        dpkg -i /tmp/dhcpcd5_9.4.1-24~deb12u4_all.deb || {
            echo_stamp "Failed to install dhcpcd5 package" ERROR
            exit 1
        }
        rm -f /tmp/dhcpcd5_9.4.1-24~deb12u4_all.deb
        my_travis_retry apt-get --fix-broken install -y || true
    }
}

echo_stamp "Configuring SSH for orangepi user"
# Enable SSH by creating /boot/ssh file (works for Orange Pi)
touch /boot/ssh
# Update sources.list for Debian Bookworm
echo "deb http://deb.debian.org/debian bookworm main contrib non-free
deb http://deb.debian.org/debian bookworm-updates main contrib non-free
deb http://security.debian.org/debian-security bookworm-security main contrib non-free" > /etc/apt/sources.list

# Remove docker.list if exists
rm -f /etc/apt/sources.list.d/docker.list
sudo apt update
# Install and configure SSH server
my_travis_retry apt-get install -y openssh-server
my_travis_retry apt-get purge -y binutils
my_travis_retry apt-get remove -y make
my_travis_retry apt-get autoremove -y
my_travis_retry apt-get install -y build-essential
# Configure SSH for orangepi user
echo_stamp "Setting up SSH access for orangepi user"
# Create .ssh directory for orangepi user
mkdir -p /home/orangepi/.ssh
chown orangepi:orangepi /home/orangepi/.ssh
chmod 700 /home/orangepi/.ssh

    # Create SSH host keys before starting service
    echo_stamp "Creating SSH host keys"
    ssh-keygen -A
    
    # Enable SSH service (use only ssh.service for modern OpenSSH)
    systemctl disable sshd.socket || true
    systemctl disable sshd.service || true
    systemctl daemon-reload || true
    systemctl enable ssh.service
    
    # Start SSH service
    systemctl start ssh.service

# Configure SSH to allow password authentication for orangepi user
echo_stamp "Configuring SSH authentication"
cat >> /etc/ssh/sshd_config << 'EOF'

# Allow password authentication for orangepi user
Match User orangepi
    PasswordAuthentication yes
    PubkeyAuthentication yes
EOF

# Set default password for orangepi user (change this in production!)
echo_stamp "Setting default password for orangepi user"
echo 'orangepi:orangepi' | chpasswd

echo_stamp "Free up space before package installation"
# Clean up any existing packages to free space
my_travis_retry apt-get clean
my_travis_retry apt-get autoremove -y
my_travis_retry apt-get autoclean
rm -rf /var/lib/apt/lists/*
rm -rf /tmp/*
rm -rf /var/tmp/*

# Check available space
echo_stamp "Available space before package installation:"
df -h

echo_stamp "Install apt keys & repos"


# Install dirmngr and setup Orange Pi ROS repository
my_travis_retry apt-get update
my_travis_retry apt-get install --no-install-recommends -y dirmngr

# Setup Orange Pi ROS Noetic repository
install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://sorrowmx.github.io/orangepi3b-ros-noetic/public-key.asc | gpg --dearmor -o /etc/apt/keyrings/orangepi-ros-noetic.gpg
chmod 0644 /etc/apt/keyrings/orangepi-ros-noetic.gpg

echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/orangepi-ros-noetic.gpg] https://sorrowmx.github.io/orangepi3b-ros-noetic/debian bookworm main" > /etc/apt/sources.list.d/ros-noetic-orangepi.list

echo_stamp "Update apt cache"

# Clean up any existing packages to free space
my_travis_retry apt-get clean
my_travis_retry apt-get autoremove -y
my_travis_retry apt-get autoclean

# Fix any broken packages before updating
echo_stamp "Fixing any broken packages before update"
my_travis_retry apt-get --fix-broken install -y || true

# TODO: FIX ERROR: /usr/bin/apt-key: 596: /usr/bin/apt-key: cannot create /dev/null: Permission denied
my_travis_retry apt-get update
# && apt upgrade -y
echo_stamp "Skip libgeographic-dev installation - using libgeographiclib-dev instead"
# Note: libgeographiclib-dev is preferred for ROS Noetic MAVROS
# The Ubuntu libgeographic-dev packages conflict with libgeographiclib-dev
# We'll use libgeographiclib-dev from Debian repositories instead
# Let's retry fetching those packages several times, just in case
echo_stamp "Software installing"

# Install packages in smaller groups to avoid running out of space
echo_stamp "Installing basic packages first"
my_travis_retry apt-get install --no-install-recommends -y \
unzip \
zip \
ipython3 \
screen \
byobu  \
nmap \
lsof \
git \
dnsmasq  \
tmux \
tree \
vim \
tcpdump \
libpoco-dev \
libzbar0 \
build-essential \
libffi-dev \
python3-pigpio \
i2c-tools \
espeak espeak-data python3-espeak \
ntpdate \
python3-dev \
python3-systemd \
python3-opencv \
python3-dateutil \
python3-docutils \
python3-yaml \
rospack-tools \
ros-noetic-catkin \
python3-pytest \
python3-pytest-cov \
catkin-tools \
libgeographic19 \
libgeographic-dev \
python3-gi \
python3-gi-cairo \
gir1.2-gstreamer-1.0 \
gstreamer1.0-tools \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
nginx \
fcgiwrap \
python3-pip \
python3-cryptography \
python3-openssl \
python3-tornado \
avahi-daemon \
avahi-utils \
libnss-mdns \
device-tree-compiler

echo_stamp "Installing ROS dependencies"
# Force remove any existing conflicting packages first (based on Ask Ubuntu solutions)
echo_stamp "Force removing conflicting packages"
dpkg -r --force-all python3-rosdistro || true
dpkg -r --force-all python3-rosdistro-modules || true
dpkg -r --force-all python3-rosdep-modules || true
my_travis_retry apt-get autoremove -y

# Clean up any broken packages
echo_stamp "Cleaning up broken packages"
my_travis_retry apt-get --fix-broken install -y || true

# Download and install packages from sorrowmx repository
echo_stamp "Downloading ROS distro packages from sorrowmx repository"
# Download python3-rosdistro-modules
wget -O /tmp/python3-rosdistro-modules_0.9.0-1_all.deb \
    "https://sorrowmx.github.io/orangepi3b-ros-noetic/debian/pool/main/arm64/python3-rosdistro-modules_0.9.0-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdistro-modules from sorrowmx repo"
}

# Download python3-rosdistro
wget -O /tmp/python3-rosdistro_0.9.0-100_all.deb \
    "https://sorrowmx.github.io/orangepi3b-ros-noetic/debian/pool/main/arm64/python3-rosdistro_0.9.0-100_all.deb" || {
    echo_stamp "Failed to download python3-rosdistro from sorrowmx repo"
}

# Download python3-rosdep-modules
wget -O /tmp/python3-rosdep-modules_0.23.1-1_all.deb \
    "https://sorrowmx.github.io/orangepi3b-ros-noetic/debian/pool/main/arm64/python3-rosdep-modules_0.23.1-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdep-modules from sorrowmx repo"
}

# Download python3-rosdep
wget -O /tmp/python3-rosdep_0.23.1-1_all.deb \
    "https://sorrowmx.github.io/orangepi3b-ros-noetic/debian/pool/main/arm64/python3-rosdep_0.23.1-1_all.deb" || {
    echo_stamp "Failed to download python3-rosdep from sorrowmx repo"
}

# Install the downloaded packages in correct order
echo_stamp "Installing downloaded ROS distro packages"
# First install python3-rosdistro-modules (main package)
if [ -f /tmp/python3-rosdistro-modules_0.9.0-1_all.deb ]; then
    dpkg -i /tmp/python3-rosdistro-modules_0.9.0-1_all.deb || {
        echo_stamp "Failed to install python3-rosdistro-modules, trying force overwrite"
        dpkg -i --force-overwrite /tmp/python3-rosdistro-modules_0.9.0-1_all.deb || true
    }
fi

# Then install python3-rosdistro (meta-package)
if [ -f /tmp/python3-rosdistro_0.9.0-100_all.deb ]; then
    dpkg -i /tmp/python3-rosdistro_0.9.0-100_all.deb || {
        echo_stamp "Failed to install python3-rosdistro, trying force overwrite"
        dpkg -i --force-overwrite /tmp/python3-rosdistro_0.9.0-100_all.deb || true
    }
fi

# Then install python3-rosdep-modules (main rosdep package)
if [ -f /tmp/python3-rosdep-modules_0.23.1-1_all.deb ]; then
    dpkg -i /tmp/python3-rosdep-modules_0.23.1-1_all.deb || {
        echo_stamp "Failed to install python3-rosdep-modules, trying force overwrite"
        dpkg -i --force-overwrite /tmp/python3-rosdep-modules_0.23.1-1_all.deb || true
    }
fi

# Finally install python3-rosdep (rosdep meta-package)
if [ -f /tmp/python3-rosdep_0.23.1-1_all.deb ]; then
    dpkg -i /tmp/python3-rosdep_0.23.1-1_all.deb || {
        echo_stamp "Failed to install python3-rosdep, trying force overwrite"
        dpkg -i --force-overwrite /tmp/python3-rosdep_0.23.1-1_all.deb || true
    }
fi

# Clean up downloaded files
rm -f /tmp/python3-rosdistro-modules_0.9.0-1_all.deb
rm -f /tmp/python3-rosdistro_0.9.0-100_all.deb
rm -f /tmp/python3-rosdep-modules_0.23.1-1_all.deb
rm -f /tmp/python3-rosdep_0.23.1-1_all.deb

# Fix any broken dependencies
my_travis_retry apt-get --fix-broken install -y || true

# Hold all ROS packages to prevent version changes
echo_stamp "Holding ROS packages to prevent version conflicts"
apt-mark hold python3-rosdistro-modules
apt-mark hold python3-rosdistro
apt-mark hold python3-rosdep-modules
apt-mark hold python3-rosdep

# Create apt preferences to pin specific versions from sorrowmx repo
echo_stamp "Creating apt preferences to pin ROS package versions"
cat > /etc/apt/preferences.d/ros-package-pinning << 'EOF'
Package: python3-rosdistro-modules
Pin: version 0.9.0-1
Pin-Priority: 1001

Package: python3-rosdistro
Pin: version 0.9.0-100
Pin-Priority: 1001

Package: python3-rosdep-modules
Pin: version 0.23.1-1
Pin-Priority: 1001

Package: python3-rosdep
Pin: version 0.23.1-1
Pin-Priority: 1001

# Prefer packages from sorrowmx repository
Package: python3-rosdistro-modules
Pin: origin sorrowmx.github.io
Pin-Priority: 1001

Package: python3-rosdistro
Pin: origin sorrowmx.github.io
Pin-Priority: 1001

Package: python3-rosdep-modules
Pin: origin sorrowmx.github.io
Pin-Priority: 1001

Package: python3-rosdep
Pin: origin sorrowmx.github.io
Pin-Priority: 1001
EOF

# ROS distro and rosdep packages are now installed directly from sorrowmx repo
echo_stamp "ROS distro and rosdep packages installed from sorrowmx repository"

echo_stamp "Installing ROS packages"

# Check if python3-rospy is available in repositories
if apt-cache policy python3-rospy | grep -q "Candidate:" && ! apt-cache policy python3-rospy | grep -q "Candidate: (none)"; then
    echo_stamp "python3-rospy found in repositories, installing normally"
    my_travis_retry apt-get install --no-install-recommends -y python3-rospy
else
    echo_stamp "python3-rospy not found in repositories, downloading from Debian archive"
    my_travis_retry wget -O /tmp/python3-rospy_1.15.15+ds-2_all.deb http://ftp.us.debian.org/debian/pool/main/r/ros-ros-comm/python3-rospy_1.15.15+ds-2_all.deb
    my_travis_retry dpkg -i /tmp/python3-rospy_1.15.15+ds-2_all.deb
    my_travis_retry apt-get --fix-broken install -y
    rm -f /tmp/python3-rospy_1.15.15+ds-2_all.deb
fi

# Try standard installation first (excluding already installed packages)
my_travis_retry apt-get install --no-install-recommends -y \
python3-rosinstall-generator \
python3-wstool \
python3-rosinstall \
python3-roslaunch \
ros-noetic-ros-core \
ros-noetic-ros-base \
ros-noetic-ros-comm \
ros-noetic-rosout \
ros-noetic-rosbash \
ros-noetic-message-generation \
ros-noetic-message-runtime \
ros-noetic-std-msgs \
ros-noetic-sensor-msgs \
ros-noetic-genmsg \
ros-noetic-led-msgs \
ros-noetic-image-transport \
ros-noetic-ros-pytest \
ros-noetic-tf \
ros-noetic-cv-bridge \
ros-noetic-geographic-msgs \
ros-noetic-eigen-conversions \
ros-noetic-tf2-geometry-msgs \
ros-noetic-mavros \
libroscpp-core-dev \
ros-noetic-rosbridge-server \
ros-noetic-mavros-extras \
ros-noetic-web-video-server \
ros-noetic-tf2-web-republisher \
liborocos-kdl1.4 \
ros-noetic-message-runtime \
libstd-msgs-dev \
libros-rosgraph-msgs-dev || {
    echo_stamp "Standard ROS installation failed, trying force overwrite method"
    # If standard installation fails, try force overwrite for problematic packages
    for pkg in python3-rosinstall python3-rosinstall-generator; do
        if dpkg -l | grep -q "^ii.*$pkg"; then
            echo_stamp "Package $pkg already installed"
        else
            echo_stamp "Force installing $pkg"
            dpkg -i --force-overwrite /var/cache/apt/archives/${pkg}*.deb || true
        fi
    done
    # Try to fix dependencies after force installation
    my_travis_retry apt-get --fix-broken install -y || true
}

# Fix any broken packages
echo_stamp "Fixing any broken packages"
my_travis_retry apt-get --fix-broken install -y || true

# Additional cleanup based on Ask Ubuntu solutions
echo_stamp "Performing additional cleanup"
my_travis_retry apt-get clean
my_travis_retry apt-get autoremove -y

# Verify ROS installation and package versions
echo_stamp "Verifying ROS installation and package versions"
if dpkg -l | grep -q "ros-noetic-ros-core"; then
    echo_stamp "ROS Noetic core packages installed successfully" SUCCESS
else
    echo_stamp "Warning: ROS Noetic core packages may not be properly installed" ERROR
    # Try one more time to fix broken packages
    echo_stamp "Attempting final fix for broken packages"
    my_travis_retry apt-get --fix-broken install -y || true
fi

# ROS packages installation completed
echo_stamp "ROS packages installation completed"

# Final cleanup
echo_stamp "Performing final cleanup"

# Clean up after package installation to free space
echo_stamp "Cleaning up after package installation"
my_travis_retry apt-get clean
my_travis_retry apt-get autoremove -y
my_travis_retry apt-get autoclean

# Deny byobu to check available updates
sed -i "s/updates_available//" /usr/share/byobu/status/status
# sed -i "s/updates_available//" /home/pi/.byobu/status

echo_stamp "Installing pip"

pip3 --version

echo_stamp "Install and enable Butterfly (web terminal)"
# Remove EXTERNALLY-MANAGED restriction for pip
rm -f /usr/lib/python3.11/EXTERNALLY-MANAGED
# Install butterfly as orangepi user in their local bin directory (without extras)
if sudo -u orangepi pip3 install --user --no-cache-dir butterfly==3.2.5; then
    echo_stamp "Butterfly installed for orangepi" "SUCCESS"
else
    echo_stamp "Butterfly install failed, retrying with latest" "ERROR"
    sudo -u orangepi pip3 install --user --no-cache-dir butterfly || true
fi
# Enable butterfly service only if installed
if [ -x "/home/orangepi/.local/bin/butterfly.server.py" ]; then
    systemctl enable butterfly.service
    echo_stamp "butterfly.service enabled"
else
    echo_stamp "Butterfly binary not found; skipping service enable" "ERROR"
fi

echo_stamp "Install ws281x library"
my_travis_retry pip3 install --prefer-binary rpi_ws281x

echo_stamp "Installing pyzbar"
my_travis_retry pip3 install pyzbar
my_travis_retry pip3 install cv-bridge
echo_stamp "Setup ROS environment"
# Create rosout symlink
mkdir -p /opt/ros/noetic/share/rosout
ln -s /opt/ros/noetic/lib/rosout/rosout /opt/ros/noetic/share/rosout/rosout

# Setup ROS environment files
cp /opt/ros/noetic/share/catkin/cmake/templates/setup.sh.in /opt/ros/noetic/setup.sh
sed -i 's/@SETUP_DIR@/\/opt\/ros\/noetic/g' /opt/ros/noetic/setup.sh
chmod +x /opt/ros/noetic/setup.sh

cp /opt/ros/noetic/share/catkin/cmake/templates/setup.bash.in /opt/ros/noetic/setup.bash
chmod +x /opt/ros/noetic/setup.bash

# Create custom setup.bash for Orange Pi
tee /opt/ros/noetic/setup.bash > /dev/null << 'EOF'
#!/usr/bin/env bash
# ROS Noetic setup.bash

# ROS environment variables
export ROS_DISTRO=noetic
export ROS_ROOT=/opt/ros/noetic
export ROS_PACKAGE_PATH=/opt/ros/noetic/share
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1

# Add ROS binaries to PATH
export PATH=/opt/ros/noetic/bin:$PATH

# Add ROS libraries to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH

# Add ROS Python packages to PYTHONPATH
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
EOF

chmod +x /opt/ros/noetic/setup.bash

echo_stamp "Installing ptvsd"
my_travis_retry pip install ptvsd
my_travis_retry pip3 install ptvsd
echo_stamp "Installing pyzbar"
my_travis_retry pip install pyzbar
my_travis_retry pip3 install pyzbar
# Create GeographicLib symlink
ln -sf /usr/lib/aarch64-linux-gnu/libGeographicLib.so.23 /usr/lib/aarch64-linux-gnu/libGeographicLib.so

echo_stamp "Add .vimrc"
cat << EOF > /home/orangepi/.vimrc
set mouse-=a
syntax on
autocmd BufNewFile,BufRead *.launch set syntax=xml
EOF

echo_stamp "Change default keyboard layout to US"
sed -i 's/XKBLAYOUT="gb"/XKBLAYOUT="us"/g' /etc/default/keyboard

echo_stamp "Setup camera OV5647"
# Create device tree overlay for OV5647
mkdir -p /tmp
cat > /tmp/opi3b-v2-ov5647-enable.dts << 'EOF'
/dts-v1/;
/plugin/;

/ {
  compatible = "rockchip,rk3566-orangepi-3b", "rockchip,rk3566";
};

&ov5647 { status = "okay"; };

&csi2_dphy_hw { status = "okay"; };
&csi2_dphy1 { status = "okay"; };

&rkisp { status = "okay"; };
&rkisp_mmu { status = "okay"; };
&rkisp_vir0 { status = "okay"; };
EOF

mkdir -p /boot/dtb/rockchip/overlay
dtc -@ -I dts -O dtb -o /boot/dtb/rockchip/overlay/rk356x-ov5647-enable.dtbo /tmp/opi3b-v2-ov5647-enable.dts

# Add overlay to orangepiEnv.txt
if [ -f /boot/orangepiEnv.txt ]; then
    sed -i '/^overlay_prefix=/a overlays=ov5647-enable' /boot/orangepiEnv.txt
else
    echo_stamp "Warning: /boot/orangepiEnv.txt not found, creating it"
    echo "overlay_prefix=rockchip" > /boot/orangepiEnv.txt
    echo "overlays=ov5647-enable" >> /boot/orangepiEnv.txt
fi

# Create camera configuration script
tee /usr/local/bin/cam-320x240.sh >/dev/null <<'EOF'
#!/bin/sh
set -e

MEDIA=/dev/media0
DEV=/dev/video0

# Wait for device nodes to appear
for i in 1 2 3 4 5; do [ -e "$MEDIA" ] && [ -e "$DEV" ] && break; sleep 1; done

# Release devices
fuser -kv /dev/video0 /dev/video1 /dev/video2 2>/dev/null || true

# Input pipeline: RAW10 640x480
media-ctl -d $MEDIA -V '"m00_b_ov5647 1-0036":0 [fmt:SGBRG10_1X10/640x480]'
media-ctl -d $MEDIA -V '"rockchip-csi2-dphy1":0 [fmt:SGBRG10_1X10/640x480]'
media-ctl -d $MEDIA -V '"rkisp-csi-subdev":0  [fmt:SGBRG10_1X10/640x480]'
media-ctl -d $MEDIA -V '"rkisp-isp-subdev":0  [fmt:SGBRG10_1X10/640x480]'

# Align ISP crops
media-ctl -d $MEDIA --set-v4l2 '"rkisp-isp-subdev":0[crop:(0,0)/640x480]'
media-ctl -d $MEDIA --set-v4l2 '"rkisp-isp-subdev":2[crop:(0,0)/640x480]'

# ISP output
media-ctl -d $MEDIA -V '"rkisp-isp-subdev":2 [fmt:YUYV8_2X8/640x480]'

# Capture node format: 320x240 NV12
v4l2-ctl -d $DEV --set-fmt-video=width=320,height=240,pixelformat=NV12 || true
EOF

chmod +x /usr/local/bin/cam-320x240.sh

# Create systemd service for camera
tee /etc/systemd/system/cam-320x240.service >/dev/null <<'EOF'
[Unit]
Description=Configure OV5647 pipeline to 640x480 -> 320x240 on boot
Wants=dev-media0.device dev-video0.device
After=dev-media0.device dev-video0.device

[Service]
Type=oneshot
ExecStart=/usr/local/bin/cam-320x240.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable cam-320x240.service

echo_stamp "Setup nginx for web interface"
# Create nginx configuration for ROS
tee /etc/nginx/sites-available/ros > /dev/null << 'EOF'
server {
    listen 80;
    server_name localhost;
    
    # Main page - redirect to coptra
    location = / {
        return 301 /coptra/;
    }
    
    # Static files from coptra/www
    location /coptra/ {
        alias /var/www/ros/coptra/;
        index index.html;
        try_files $uri $uri/ =404;
    }
    
    # Static files from coptra_blocks/www
    location /coptra_blocks/ {
        alias /var/www/ros/coptra_blocks/;
        index index.html;
        try_files $uri $uri/ =404;
    }
    
    # For static files (CSS, JS, images)
    location ~* \.(css|js|png|jpg|jpeg|gif|ico|svg)$ {
        root /var/www/ros;
        expires 1y;
        add_header Cache-Control "public, immutable";
    }
    
    # CGI scripts for network management
    location /cgi-bin/ {
        alias /usr/lib/cgi-bin/;
        gzip off;
        fastcgi_pass unix:/var/run/fcgiwrap.socket;
        include /etc/nginx/fastcgi_params;
        fastcgi_param SCRIPT_FILENAME /usr/lib/cgi-bin$fastcgi_script_name;
    }
}
EOF

# Create symbolic link and remove default
ln -s /etc/nginx/sites-available/ros /etc/nginx/sites-enabled/
rm -f /etc/nginx/sites-enabled/default

# Create directory for ROS web files
mkdir -p /var/www/ros
if [ -d /home/orangepi/.ros/www ]; then
    cp -r /home/orangepi/.ros/www/* /var/www/ros/
else
    echo_stamp "Warning: /home/orangepi/.ros/www not found, creating empty directory"
    mkdir -p /var/www/ros/coptra
fi
chown -R www-data:www-data /var/www/ros
chmod -R 755 /var/www/ros

# Setup CGI for network management
echo_stamp "Setup CGI scripts for network management"

# Install and enable fcgiwrap
systemctl enable fcgiwrap
systemctl start fcgiwrap

# Create CGI directory
mkdir -p /usr/lib/cgi-bin

# Create network switch CGI script
cat << 'EOF' > /usr/lib/cgi-bin/network-switch
#!/bin/bash

echo "Content-Type: application/json"
echo ""

# Read POST data
if [ "$REQUEST_METHOD" = "POST" ]; then
    read -r POST_DATA
    MODE=$(echo "$POST_DATA" | grep -o '"mode":"[^"]*"' | cut -d'"' -f4)
    
    if [ -n "$MODE" ]; then
        # Execute network switch command
        /usr/local/bin/network-switch "$MODE" > /dev/null 2>&1
        
        if [ $? -eq 0 ]; then
            echo '{"status": "success", "message": "Mode switched successfully"}'
        else
            echo '{"status": "error", "message": "Failed to switch mode"}'
        fi
    else
        echo '{"status": "error", "message": "Invalid mode parameter"}'
    fi
else
    echo '{"status": "error", "message": "Method not allowed"}'
fi
EOF

# Create network status CGI script
cat << 'EOF' > /usr/lib/cgi-bin/network-status
#!/bin/bash

echo "Content-Type: application/json"
echo ""

# Get current network status
CURRENT_MODE="unknown"
WIFI_STATUS="unknown"
ETH_STATUS="unknown"
IP_ADDRESS="unknown"

# Check current mode by looking at running services
if systemctl is-active --quiet hostapd && systemctl is-active --quiet dnsmasq; then
    CURRENT_MODE="ap"
elif systemctl is-active --quiet NetworkManager; then
    CURRENT_MODE="client"
else
    CURRENT_MODE="eth0"
fi

# Check WiFi status
if ip link show wlan0 | grep -q "UP"; then
    WIFI_STATUS="UP"
else
    WIFI_STATUS="DOWN"
fi

# Check Ethernet status
if ip link show end1 | grep -q "UP"; then
    ETH_STATUS="UP"
else
    ETH_STATUS="DOWN"
fi

# Get IP address
IP_ADDRESS=$(ip addr show wlan0 | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
if [ -z "$IP_ADDRESS" ]; then
    IP_ADDRESS=$(ip addr show end1 | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
fi

# Get current SSID
CURRENT_SSID="unknown"
if [ -f /etc/hostapd/hostapd.conf ]; then
    CURRENT_SSID=$(grep "^ssid=" /etc/hostapd/hostapd.conf | cut -d'=' -f2 | tr -d '"')
elif [ -f /boot/wpa_supplicant.conf ]; then
    CURRENT_SSID=$(grep "ssid=" /boot/wpa_supplicant.conf | head -1 | cut -d'"' -f2)
fi

# Output JSON
cat << JSON_EOF
{
    "currentMode": "$CURRENT_MODE",
    "wifiStatus": "$WIFI_STATUS",
    "ethStatus": "$ETH_STATUS",
    "ipAddress": "$IP_ADDRESS",
    "currentSSID": "$CURRENT_SSID"
}
JSON_EOF
EOF

# Create restart network CGI script
cat << 'EOF' > /usr/lib/cgi-bin/restart-network
#!/bin/bash

echo "Content-Type: application/json"
echo ""

if [ "$REQUEST_METHOD" = "POST" ]; then
    # Restart network services
    systemctl restart hostapd dnsmasq NetworkManager > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo '{"status": "success", "message": "Network services restarted successfully"}'
    else
        echo '{"status": "error", "message": "Failed to restart network services"}'
    fi
else
    echo '{"status": "error", "message": "Method not allowed"}'
fi
EOF

# Make CGI scripts executable
chmod +x /usr/lib/cgi-bin/network-switch
chmod +x /usr/lib/cgi-bin/network-status
chmod +x /usr/lib/cgi-bin/restart-network

echo_stamp "Attempting to kill dirmngr"
gpgconf --kill dirmngr
# dirmngr is only used by apt-key, so we can safely kill it.
# We ignore pkill's exit value as well.
pkill -9 -f dirmngr || true

echo_stamp "Cleaning up to free space"
# Clean up package cache and temporary files
my_travis_retry apt-get clean
my_travis_retry apt-get autoremove -y
my_travis_retry apt-get autoclean
rm -rf /var/lib/apt/lists/*
rm -rf /tmp/*
rm -rf /var/tmp/*

echo_stamp "End of software installation"
