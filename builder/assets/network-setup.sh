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
ip route add 192.168.11.0/24 dev wlan0 2>/dev/null || true

# Clear ARP cache to prevent stale entries
ip neigh flush all 2>/dev/null || true

# Ensure interface is properly up
ip link set wlan0 up 2>/dev/null || true

echo_stamp "Network routing configured successfully"

# Apply AP/DHCP fixes only if hostapd is intended to run (AP mode)
CHANGED=false
if systemctl is-active --quiet hostapd; then
    # Ensure dnsmasq config for AP exists and is sane
    if [ ! -f /etc/dnsmasq.d/coptra.conf ]; then
    echo_stamp "Creating /etc/dnsmasq.d/coptra.conf"
    mkdir -p /etc/dnsmasq.d
    cat << 'EOF' > /etc/dnsmasq.d/coptra.conf
interface=wlan0
bind-interfaces
except-interface=end1
address=/coptra/192.168.11.1
dhcp-authoritative
domain-needed
bogus-priv
no-hosts
quiet-dhcp6
dhcp-range=192.168.11.100,192.168.11.200,12h
dhcp-option=3,192.168.11.1
dhcp-option=6,192.168.11.1
EOF
    CHANGED=true
    else
    echo_stamp "dnsmasq AP config already present"
    fi

    # Comment out conflicting directives in /etc/dnsmasq.conf if any
    if grep -qE '^(interface|dhcp-|listen-address)' /etc/dnsmasq.conf 2>/dev/null; then
    echo_stamp "Sanitizing /etc/dnsmasq.conf directives"
    sed -i 's/^\(interface\|dhcp-\|listen-address\)/# &/' /etc/dnsmasq.conf || true
    CHANGED=true
    fi

    # Ensure wlan0 static IP in dhcpcd.conf
    if ! grep -qE '^[[:space:]]*interface[[:space:]]+wlan0$' /etc/dhcpcd.conf 2>/dev/null; then
        printf "interface wlan0\nstatic ip_address=192.168.11.1/24\n" >> /etc/dhcpcd.conf
        echo_stamp "Added wlan0 static IP to /etc/dhcpcd.conf"
        CHANGED=true
    else
        if ! grep -qE '^[[:space:]]*static[[:space:]]+ip_address=192\.168\.11\.1/24$' /etc/dhcpcd.conf; then
            printf "static ip_address=192.168.11.1/24\n" >> /etc/dhcpcd.conf
            echo_stamp "Appended static IP under existing wlan0 stanza"
            CHANGED=true
        fi
    fi

    # Ensure interface IP only if missing
    if ! ip addr show wlan0 | grep -q "inet 192.168.11.1/24"; then
        ip addr flush dev wlan0 || true
        ip link set wlan0 up || true
        ip addr add 192.168.11.1/24 dev wlan0 2>/dev/null || true
        CHANGED=true
    fi

    # Check if dnsmasq is listening on DHCP
    if ! ss -ulpn 2>/dev/null | grep -q ":67"; then
        CHANGED=true
    fi

    # Apply restarts only if something changed
    if [ "$CHANGED" = true ]; then
        systemctl stop NetworkManager 2>/dev/null || true
        systemctl restart dhcpcd 2>/dev/null || true
        systemctl restart hostapd 2>/dev/null || true
        systemctl restart dnsmasq 2>/dev/null || true
        echo_stamp "AP services restarted (hostapd/dnsmasq)"
    else
        echo_stamp "No changes detected; skipping service restarts"
    fi
else
    echo_stamp "hostapd not active; skipping AP/DHCP adjustments (client/eth mode)"
fi

# Check if hostapd is running and configure if needed
if systemctl is-active --quiet hostapd; then
    echo_stamp "hostapd is running, network setup complete"
else
    echo_stamp "hostapd not running, this is normal for client mode"
fi

echo_stamp "Network setup service completed"
