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
for tool in losetup parted resize2fs partprobe; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Error: Required tool '$tool' not found"
        exit 1
    fi
done

# Check for growpart (alternative method)
HAS_GROWPART=false
if command -v growpart >/dev/null 2>&1; then
    HAS_GROWPART=true
    echo_stamp "growpart available - will use as fallback"
fi

# Check for qemu-img (preferred method)
HAS_QEMU_IMG=false
if command -v qemu-img >/dev/null 2>&1; then
    HAS_QEMU_IMG=true
    echo_stamp "qemu-img available - will use for image resize"
fi

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
    
    # Use qemu-img if available (more reliable)
    if [ "$HAS_QEMU_IMG" = true ]; then
        echo_stamp "Using qemu-img to resize image"
        qemu-img resize "$IMAGE_PATH" "$TARGET_SIZE"
    else
        # Fallback to dd method
        echo_stamp "Using dd to resize image"
        CHUNK_SIZE=1024  # 1GB chunks
        REMAINING_MB=$ADD_MB
        
        while [ $REMAINING_MB -gt 0 ]; do
            CURRENT_CHUNK=$((REMAINING_MB > CHUNK_SIZE ? CHUNK_SIZE : REMAINING_MB))
            echo_stamp "Adding chunk: ${CURRENT_CHUNK}MB (${REMAINING_MB}MB remaining)"
            
            dd if=/dev/zero bs=1M count=$CURRENT_CHUNK >> "$IMAGE_PATH" 2>/dev/null
            REMAINING_MB=$((REMAINING_MB - CURRENT_CHUNK))
        done
    fi
    
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

# Force kernel to re-read partition table
partprobe "$LOOP_DEVICE" 2>/dev/null || true
sleep 1

# Find the partition (usually the root partition is p2)
PARTITION_DEVICE=""
if [ -b "${LOOP_DEVICE}p2" ]; then
    PARTITION_DEVICE="${LOOP_DEVICE}p2"
    echo_stamp "Found root partition: $PARTITION_DEVICE"
elif [ -b "${LOOP_DEVICE}p1" ]; then
    PARTITION_DEVICE="${LOOP_DEVICE}p1"
    echo_stamp "Found partition: $PARTITION_DEVICE"
else
    echo_stamp "No partitions found, checking if entire device has filesystem"
    if file -s "$LOOP_DEVICE" | grep -q "ext4\|ext3\|ext2"; then
        PARTITION_DEVICE="$LOOP_DEVICE"
        echo_stamp "Using entire device as filesystem: $PARTITION_DEVICE"
    else
        echo_stamp "Error: No partition or filesystem found"
        losetup -d "$LOOP_DEVICE"
        exit 1
    fi
fi

# Resize the partition (only if we have partitions)
if [ "$PARTITION_DEVICE" != "$LOOP_DEVICE" ]; then
    echo_stamp "Resizing partition"
    
    # Determine which partition to resize
    PARTITION_NUM=""
    if [ -b "${LOOP_DEVICE}p2" ]; then
        PARTITION_NUM="2"
    elif [ -b "${LOOP_DEVICE}p1" ]; then
        PARTITION_NUM="1"
    fi

    if [ -n "$PARTITION_NUM" ]; then
        echo_stamp "Resizing partition $PARTITION_NUM"
        
        # Use growpart (most reliable for GPT)
        if [ "$HAS_GROWPART" = true ]; then
            echo_stamp "Using growpart to resize partition"
            if growpart "$LOOP_DEVICE" "$PARTITION_NUM"; then
                echo_stamp "growpart succeeded"
            else
                echo_stamp "growpart failed, trying parted"
                if ! parted "$LOOP_DEVICE" resizepart "$PARTITION_NUM" 100% --script; then
                    echo_stamp "Warning: Partition resize failed, but continuing with filesystem resize"
                fi
            fi
        else
            echo_stamp "Using parted to resize partition"
            if ! parted "$LOOP_DEVICE" resizepart "$PARTITION_NUM" 100% --script; then
                echo_stamp "Warning: Partition resize failed, but continuing with filesystem resize"
            fi
        fi
    else
        echo_stamp "Error: Could not determine partition number"
        losetup -d "$LOOP_DEVICE"
        exit 1
    fi
else
    echo_stamp "Using entire device, skipping partition resize"
fi

# Wait for partition table to be updated and force re-read
sleep 2
partprobe "$LOOP_DEVICE" 2>/dev/null || true
sleep 1

# Resize the filesystem
echo_stamp "Resizing filesystem on: $PARTITION_DEVICE"
echo_stamp "Filesystem size before resize:"
df -h "$PARTITION_DEVICE" 2>/dev/null || echo "Cannot check filesystem size"

# Check if the device has a filesystem
if file -s "$PARTITION_DEVICE" | grep -q "ext4\|ext3\|ext2"; then
    echo_stamp "Filesystem detected, resizing..."
    if resize2fs "$PARTITION_DEVICE"; then
        echo_stamp "Filesystem resized successfully"
        echo_stamp "Filesystem size after resize:"
        df -h "$PARTITION_DEVICE" 2>/dev/null || echo "Cannot check filesystem size"
    else
        echo_stamp "Warning: Filesystem resize failed, but continuing..."
    fi
else
    echo_stamp "No filesystem detected on $PARTITION_DEVICE, skipping resize"
fi

# Clean up
echo_stamp "Cleaning up loop device"
losetup -d "$LOOP_DEVICE"

echo_stamp "Filesystem resize complete"
