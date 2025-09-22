#!/bin/bash

# Script for working with image files in chroot environment
# Usage: image-chroot.sh <IMAGE_PATH> <COMMAND> [ARGS...]
# Commands: copy, exec

set -e

IMAGE_PATH="$1"
COMMAND="$2"
shift 2

if [ -z "$IMAGE_PATH" ] || [ -z "$COMMAND" ]; then
    echo "Usage: $0 <IMAGE_PATH> <COMMAND> [ARGS...]"
    echo "Commands: copy <SOURCE> <DEST>, exec <SCRIPT> [ARGS...]"
    exit 1
fi

if [ ! -f "$IMAGE_PATH" ]; then
    echo "Error: Image file $IMAGE_PATH not found"
    exit 1
fi

# Create temporary mount point
TEMP_DIR=$(mktemp -d)
BOOT_DIR="$TEMP_DIR/boot"

echo_stamp() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

echo_stamp "Mount loop-image: $IMAGE_PATH"

# Mount the image
LOOP_DEV=$(losetup --show -f -P "$IMAGE_PATH")
echo_stamp "Loop device: $LOOP_DEV"

# Wait for loop device to be ready
sleep 2

# Mount root filesystem
mount "${LOOP_DEV}p2" "$TEMP_DIR"
echo_stamp "Mounted root filesystem"

# Mount boot partition if it exists
if [ -b "${LOOP_DEV}p1" ]; then
    mount "${LOOP_DEV}p1" "$BOOT_DIR"
    echo_stamp "Mounted boot partition"
fi

# Setup QEMU for ARM64 emulation
if [ -f /usr/bin/qemu-aarch64-static ]; then
    cp /usr/bin/qemu-aarch64-static "$TEMP_DIR/usr/bin/"
    echo_stamp "Copied QEMU aarch64 static"
fi

# Bind mount necessary directories
mount --bind /dev "$TEMP_DIR/dev"
mount --bind /proc "$TEMP_DIR/proc"
mount --bind /sys "$TEMP_DIR/sys"

# Function to cleanup
cleanup() {
    echo_stamp "Cleaning up..."
    umount "$TEMP_DIR/dev" 2>/dev/null || true
    umount "$TEMP_DIR/proc" 2>/dev/null || true
    umount "$TEMP_DIR/sys" 2>/dev/null || true
    umount "$BOOT_DIR" 2>/dev/null || true
    umount "$TEMP_DIR" 2>/dev/null || true
    losetup -d "$LOOP_DEV" 2>/dev/null || true
    rm -rf "$TEMP_DIR"
    echo_stamp "Cleanup complete"
}

# Set trap for cleanup
trap cleanup EXIT

# Execute command
case "$COMMAND" in
    "copy")
        SOURCE="$1"
        DEST="$2"
        if [ -z "$SOURCE" ] || [ -z "$DEST" ]; then
            echo "Error: copy requires SOURCE and DEST arguments"
            exit 1
        fi
        echo_stamp "Copying $SOURCE to $DEST"
        cp -r "$SOURCE" "$TEMP_DIR$DEST"
        echo_stamp "Copy complete"
        ;;
    "exec")
        SCRIPT="$1"
        shift
        if [ -z "$SCRIPT" ]; then
            echo "Error: exec requires SCRIPT argument"
            exit 1
        fi
        echo_stamp "Executing script: $SCRIPT"
        
        # Copy script to chroot if it's not already there
        SCRIPT_NAME=$(basename "$SCRIPT")
        if [ ! -f "$TEMP_DIR$SCRIPT" ]; then
            echo_stamp "Copying script $SCRIPT to chroot"
            # Create directory structure if it doesn't exist
            mkdir -p "$(dirname "$TEMP_DIR$SCRIPT")"
            cp "$SCRIPT" "$TEMP_DIR$SCRIPT"
            chmod +x "$TEMP_DIR$SCRIPT"
        fi
        
        # Use QEMU for ARM64 emulation if available
        if [ -f "$TEMP_DIR/usr/bin/qemu-aarch64-static" ]; then
            chroot "$TEMP_DIR" /usr/bin/qemu-aarch64-static /bin/bash "$SCRIPT" "$@"
        else
            chroot "$TEMP_DIR" /bin/bash "$SCRIPT" "$@"
        fi
        echo_stamp "Script execution complete"
        ;;
    *)
        echo "Error: Unknown command '$COMMAND'"
        echo "Available commands: copy, exec"
        exit 1
        ;;
esac

echo_stamp "Operation completed successfully"
