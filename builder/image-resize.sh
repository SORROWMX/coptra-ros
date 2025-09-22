#!/bin/bash

# Script for resizing image files
# Usage: image-resize.sh <IMAGE_PATH> [SIZE] [MODE]
# SIZE: target size (e.g., 7G, 8G, 10G)
# MODE: 'max' for maximum size, 'min' for minimum size

set -e

IMAGE_PATH="$1"
TARGET_SIZE="${2:-10G}"
MODE="${3:-max}"

if [ -z "$IMAGE_PATH" ]; then
    echo "Usage: $0 <IMAGE_PATH> [SIZE] [MODE]"
    echo "Example: $0 image.img 10G max"
    exit 1
fi

if [ ! -f "$IMAGE_PATH" ]; then
    echo "Error: Image file $IMAGE_PATH not found"
    exit 1
fi

# Check for required tools
for tool in losetup parted resize2fs; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Error: Required tool '$tool' not found"
        exit 1
    fi
done

echo_stamp() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

echo_stamp "Resizing image: $IMAGE_PATH to $TARGET_SIZE"

# Get current size
CURRENT_SIZE=$(stat -c%s "$IMAGE_PATH")
echo_stamp "Current size: $((CURRENT_SIZE / 1024 / 1024)) MB"

# Parse target size
if [ "$TARGET_SIZE" = "max" ]; then
    # For max mode, we'll calculate the size based on current size + buffer
    TARGET_BYTES=$((CURRENT_SIZE + 3 * 1024 * 1024 * 1024))  # Add 3GB buffer
elif [[ "$TARGET_SIZE" =~ ^([0-9]+)([GMK]?)$ ]]; then
    SIZE_NUM="${BASH_REMATCH[1]}"
    SIZE_UNIT="${BASH_REMATCH[2]}"
    
    case "$SIZE_UNIT" in
        "G"|"")
            TARGET_BYTES=$((SIZE_NUM * 1024 * 1024 * 1024))
            ;;
        "M")
            TARGET_BYTES=$((SIZE_NUM * 1024 * 1024))
            ;;
        "K")
            TARGET_BYTES=$((SIZE_NUM * 1024))
            ;;
    esac
else
    echo "Error: Invalid size format: $TARGET_SIZE"
    echo "Valid formats: 7G, 8G, 10G, max"
    exit 1
fi

echo_stamp "Target size: $((TARGET_BYTES / 1024 / 1024)) MB"

# Calculate how much to add
if [ "$MODE" = "max" ] || [ "$TARGET_SIZE" = "max" ]; then
    # Resize to target size
    ADD_SIZE=$((TARGET_BYTES - CURRENT_SIZE))
else
    # For ROS installation, we need at least 3GB free space
    MIN_FREE_SPACE=$((3 * 1024 * 1024 * 1024))  # 3GB
    ADD_SIZE=$((MIN_FREE_SPACE - (CURRENT_SIZE % (1024 * 1024 * 1024))))
    if [ $ADD_SIZE -lt $((1024 * 1024 * 1024)) ]; then
        ADD_SIZE=$((1024 * 1024 * 1024))  # At least 1GB
    fi
fi

if [ $ADD_SIZE -gt 0 ]; then
    ADD_MB=$((ADD_SIZE / 1024 / 1024))
    echo_stamp "Adding $ADD_MB MB to image"
    
    # Resize the image in chunks to avoid memory issues
    CHUNK_SIZE=1024  # 1GB chunks
    REMAINING_MB=$ADD_MB
    
    while [ $REMAINING_MB -gt 0 ]; do
        CURRENT_CHUNK=$((REMAINING_MB > CHUNK_SIZE ? CHUNK_SIZE : REMAINING_MB))
        echo_stamp "Adding chunk: ${CURRENT_CHUNK}MB (${REMAINING_MB}MB remaining)"
        
        dd if=/dev/zero bs=1M count=$CURRENT_CHUNK >> "$IMAGE_PATH" 2>/dev/null
        REMAINING_MB=$((REMAINING_MB - CURRENT_CHUNK))
    done
    
    echo_stamp "Image resized"
else
    echo_stamp "Image is already larger than target size"
fi

# Get new size
NEW_SIZE=$(stat -c%s "$IMAGE_PATH")
echo_stamp "New size: $((NEW_SIZE / 1024 / 1024)) MB"

echo_stamp "Image resize complete"

# Now we need to resize the filesystem inside the image
echo_stamp "Resizing filesystem inside the image"

# Find the loop device for the image
LOOP_DEVICE=$(losetup -f --show "$IMAGE_PATH")
echo_stamp "Using loop device: $LOOP_DEVICE"

# Wait a moment for the loop device to be ready
sleep 2

# Find the partition (usually the second partition)
PARTITION_DEVICE="${LOOP_DEVICE}p2"
if [ ! -b "$PARTITION_DEVICE" ]; then
    PARTITION_DEVICE="${LOOP_DEVICE}p1"
fi

echo_stamp "Using partition: $PARTITION_DEVICE"

# Resize the partition using parted
echo_stamp "Resizing partition table"
parted "$LOOP_DEVICE" resizepart 2 100% || parted "$LOOP_DEVICE" resizepart 1 100%

# Wait for partition table to be updated
sleep 2

# Resize the filesystem
echo_stamp "Resizing filesystem"
resize2fs "$PARTITION_DEVICE"

# Clean up
echo_stamp "Cleaning up loop device"
losetup -d "$LOOP_DEVICE"

echo_stamp "Filesystem resize complete"
