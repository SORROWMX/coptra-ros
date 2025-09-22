#! /usr/bin/env bash

#
# Script for network configuration
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

echo_stamp "#1 Write STATIC to /etc/dhcpcd.conf"

cat << EOF >> /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.11.1/24
EOF

echo_stamp "#2 Set wpa_supplicant country"

mkdir -p /etc/wpa_supplicant
if [ -f /etc/wpa_supplicant/wpa_supplicant.conf ]; then
    cat << EOF >> /etc/wpa_supplicant/wpa_supplicant.conf
country=GB
EOF
else
    cat << EOF > /etc/wpa_supplicant/wpa_supplicant.conf
country=GB
EOF
fi

echo_stamp "#3 Write dhcp-config to /etc/dnsmasq.conf"

cat << EOF >> /etc/dnsmasq.conf
interface=wlan0
address=/coptra/192.168.11.1
dhcp-range=192.168.11.100,192.168.11.200,12h
no-hosts
filterwin2k
bogus-priv
domain-needed
quiet-dhcp6
EOF

echo_stamp "#4 Configure NetworkManager to exclude wlan0"

# Create NetworkManager config to exclude wlan0 from management
mkdir -p /etc/NetworkManager/conf.d
cat << 'EOF' > /etc/NetworkManager/conf.d/99-unmanaged-devices.conf
[keyfile]
unmanaged-devices=interface-name:wlan0
EOF

echo_stamp "#5 Create WiFi mode switcher script"

# Create network mode switcher script
cat << 'EOF' > /usr/local/bin/network-switch
#!/bin/bash

# Get current SSID from hostapd config or generate new one
get_current_ssid() {
    if [ -f /etc/hostapd/hostapd.conf ]; then
        grep "^ssid=" /etc/hostapd/hostapd.conf | cut -d'=' -f2 | tr -d '"'
    else
        # Generate new SSID if config doesn't exist
        echo "coptra-$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)"
    fi
}

show_status() {
    echo "=== Network Status ==="
    echo "WiFi Interface:"
    iw dev wlan0 info 2>/dev/null || echo "  wlan0 not available"
    ip addr show wlan0 2>/dev/null | grep -E "(inet|UP|DOWN)"
    echo ""
    echo "Ethernet Interface:"
    ip addr show end1 2>/dev/null | grep -E "(inet|UP|DOWN)"
    echo ""
    echo "Services:"
    systemctl is-active hostapd dnsmasq NetworkManager 2>/dev/null | paste - <(echo -e "hostapd\ndnsmasq\nNetworkManager")
    echo ""
    echo "Current SSID: $(get_current_ssid)"
}

case "$1" in
    ap) 
        echo "Switching to WiFi Access Point mode..."
        systemctl stop NetworkManager
        systemctl start hostapd dnsmasq
        ip addr add 192.168.11.1/24 dev wlan0 2>/dev/null || true
        CURRENT_SSID=$(get_current_ssid)
        echo "✅ Access Point mode active"
        echo "   SSID: $CURRENT_SSID"
        echo "   Password: coptrawifi"
        echo "   Web interface: http://192.168.11.1"
        echo "   Ethernet: Available for internet sharing"
        ;;
    client) 
        echo "Switching to WiFi Client mode..."
        systemctl stop hostapd dnsmasq
        ip addr del 192.168.11.1/24 dev wlan0 2>/dev/null || true
        systemctl start NetworkManager
        echo "✅ WiFi Client mode active"
        echo "   Use NetworkManager to connect to WiFi networks"
        echo "   Ethernet: Available for internet sharing"
        ;;
    eth0)
        echo "Switching to Ethernet-only mode..."
        systemctl stop hostapd dnsmasq NetworkManager
        ip addr del 192.168.11.1/24 dev wlan0 2>/dev/null || true
        echo "✅ Ethernet-only mode active"
        echo "   WiFi disabled"
        echo "   Ethernet: Available for internet sharing"
        ;;
    status)
        show_status
        ;;
    *) 
        echo "Usage: network-switch {ap|client|eth0|status}"
        echo ""
        echo "Modes:"
        echo "  ap      - WiFi Access Point (SSID: coptra-XXXX)"
        echo "  client  - WiFi Client (connect to existing networks)"
        echo "  eth0    - Ethernet only (WiFi disabled)"
        echo "  status  - Show current network status"
        echo ""
        echo "Examples:"
        echo "  network-switch ap     # Start WiFi hotspot"
        echo "  network-switch client # Connect to WiFi network"
        echo "  network-switch eth0   # Use only Ethernet"
        ;;
esac
EOF

chmod +x /usr/local/bin/network-switch

echo_stamp "#7 Enable services and set initial mode"

# Enable services
systemctl enable hostapd
systemctl enable dnsmasq
systemctl enable dhcpcd

# Set initial mode to Access Point
systemctl stop NetworkManager
systemctl start hostapd dnsmasq
ip addr add 192.168.11.1/24 dev wlan0 2>/dev/null || true

echo_stamp "#8 End of network installation"
