#!/bin/bash

# Network Setup Script for Coptra
# This script runs once on boot to configure network routing

set -e

echo_stamp() {
    TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
    TEXT="\e[1m$TEXT\e[0m"
    echo -e ${TEXT}
}

# Check if wlan0 interface exists
if ! ip link show wlan0 >/dev/null 2>&1; then
    echo_stamp "Warning: wlan0 interface not found, skipping network setup"
    exit 0
fi

echo_stamp "Configuring network routing for better client connectivity"

# Ensure proper routing table
ip route del 192.168.11.0/24 dev wlan0 2>/dev/null || true
ip route add 192.168.11.0/24 dev wlan0

# Clear ARP cache to prevent stale entries
ip neigh flush all

# Ensure interface is properly up
ip link set wlan0 up

echo_stamp "Network routing configured successfully"

# Check if hostapd is running and configure if needed
if systemctl is-active --quiet hostapd; then
    echo_stamp "hostapd is running, network setup complete"
else
    echo_stamp "hostapd not running, this is normal for client mode"
fi

echo_stamp "Network setup service completed"
