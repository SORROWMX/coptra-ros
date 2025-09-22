#!/bin/bash

# Script for resizing image files
# Usage: image-resize.sh <IMAGE_PATH>

set -e

IMAGE_PATH="$1"

if [ -z "$IMAGE_PATH" ]; then
    echo "Usage: $0 <IMAGE_PATH>"
    exit 1
fi

if [ ! -f "$IMAGE_PATH" ]; then
    echo "Error: Image file $IMAGE_PATH not found"
    exit 1
fi

echo_stamp() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

echo_stamp "Resizing image: $IMAGE_PATH"

# Get current size
CURRENT_SIZE=$(stat -c%s "$IMAGE_PATH")
echo_stamp "Current size: $((CURRENT_SIZE / 1024 / 1024)) MB"

# Resize to minimum size (add 100MB buffer)
MIN_SIZE=$((CURRENT_SIZE + 100 * 1024 * 1024))
echo_stamp "Target size: $((MIN_SIZE / 1024 / 1024)) MB"

# Resize the image
dd if=/dev/zero bs=1M count=100 >> "$IMAGE_PATH" 2>/dev/null
echo_stamp "Image resized"

# Get new size
NEW_SIZE=$(stat -c%s "$IMAGE_PATH")
echo_stamp "New size: $((NEW_SIZE / 1024 / 1024)) MB"

echo_stamp "Image resize complete"
