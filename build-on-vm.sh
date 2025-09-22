#!/bin/bash

# Script for building Orange Pi 3B image on virtual machine
# Usage: ./build-on-vm.sh

set -e

echo "Starting Orange Pi 3B Coptra image build on VM"
echo "=============================================="

# Check root privileges
if [ "$EUID" -ne 0 ]; then
    echo "Error: Run script with root privileges: sudo ./build-on-vm.sh"
    exit 1
fi

# Check if we're in the right directory
if [ ! -d "builder" ] || [ ! -f "builder/image-build.sh" ]; then
    echo "Error: builder directory not found!"
    echo "Make sure you're running this script from the repository root directory"
    echo "Current directory: $(pwd)"
    echo "Expected files:"
    echo "  - builder/image-build.sh"
    echo "  - builder/image-chroot.sh"
    echo "  - builder/image-software.sh"
    echo "  - etc."
    exit 1
fi

# Check required commands availability
echo "Checking dependencies..."
for cmd in qemu-aarch64-static kpartx parted unzip wget curl git; do
    if ! command -v $cmd &> /dev/null; then
        echo "Error: Command $cmd not found. Install dependencies:"
        echo "   apt install -y qemu-user-static kpartx parted unzip wget curl git"
        exit 1
    fi
done

# Check binfmt
if ! update-binfmts --status | grep -q "qemu-aarch64"; then
    echo "Warning: Setting up binfmt for QEMU..."
    update-binfmts --enable qemu-aarch64
fi

# Set environment variables
export DEBIAN_FRONTEND=noninteractive
export TZ=UTC
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# Check available disk space
AVAILABLE_SPACE=$(df . | tail -1 | awk '{print $4}')
if [ "$AVAILABLE_SPACE" -lt 10485760 ]; then  # 10GB in KB
    echo "Error: Not enough free space. Need at least 10GB"
    echo "   Available: $((AVAILABLE_SPACE / 1024 / 1024))GB"
    exit 1
fi

echo "Dependencies checked successfully"
echo "Free space: $((AVAILABLE_SPACE / 1024 / 1024))GB"

# Create images directory if not exists
mkdir -p images

# Clean up old loop devices
echo "Cleaning up old loop devices..."
losetup -D 2>/dev/null || true

# Make scripts executable
echo "Setting up scripts..."
chmod +x builder/*.sh

# Verify scripts are executable
echo "Verifying builder scripts..."
for script in builder/image-build.sh builder/image-chroot.sh builder/image-software.sh builder/image-network.sh builder/image-ros.sh builder/image-validate.sh builder/image-resize.sh; do
    if [ -f "$script" ]; then
        echo "  ✓ $script"
    else
        echo "  ✗ $script (missing)"
        exit 1
    fi
done

# Check for existing images
echo "Checking for existing images..."
if ls images/*.img 1> /dev/null 2>&1; then
    echo "Found existing images:"
    ls -lh images/*.img
    echo ""
    echo "Note: If you want to force rebuild, use: FORCE_REBUILD=true sudo ./build-on-vm.sh"
    echo ""
fi

# Start build process
echo "Starting image build..."
echo "   This may take 1-3 hours depending on VM performance"
echo "   (Will skip download if image already exists)"
echo ""

# Run main build script
./builder/image-build.sh

# Check result
if [ $? -eq 0 ]; then
    echo ""
    echo "Build completed successfully!"
    echo "Image is located in images/ directory"
    ls -la images/*.img 2>/dev/null || echo "   No images found"
    
    # Offer to compress image
    echo ""
    read -p "Compress image to save space? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Compressing image..."
        for img in images/*.img; do
            if [ -f "$img" ]; then
                echo "   Compressing $img..."
                xz -9 "$img"
                echo "   Done: ${img}.xz"
            fi
        done
    fi
    
    echo ""
    echo "Next steps:"
    echo "   1. Download image from VM"
    echo "   2. Flash to microSD card using balenaEtcher"
    echo "   3. Insert card into Orange Pi 3B and power on"
    echo "   4. Connect to WiFi 'coptra-XXXX' (password: coptrawifi)"
    echo "   5. Open http://192.168.11.1 in browser"
    
else
    echo ""
    echo "Build failed with error!"
    echo "Check logs above for diagnostics"
    exit 1
fi
