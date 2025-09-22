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
QEMU_BINARY=""
for qemu_path in /usr/bin/qemu-aarch64-static /usr/bin/qemu-arm-static /usr/bin/qemu-aarch64 /usr/bin/qemu-arm /usr/bin/qemu-aarch64-static /usr/bin/qemu-arm-static; do
    if [ -f "$qemu_path" ]; then
        QEMU_BINARY="$qemu_path"
        echo_stamp "Found QEMU binary: $qemu_path"
        break
    fi
done

# Try to install QEMU if not found
if [ -z "$QEMU_BINARY" ]; then
    echo_stamp "QEMU binary not found, attempting to install..."
    if command -v apt-get >/dev/null 2>&1; then
        apt-get update -qq && apt-get install -y qemu-user-static binfmt-support
        # Try again after installation
        for qemu_path in /usr/bin/qemu-aarch64-static /usr/bin/qemu-arm-static /usr/bin/qemu-aarch64 /usr/bin/qemu-arm; do
            if [ -f "$qemu_path" ]; then
                QEMU_BINARY="$qemu_path"
                echo_stamp "Found QEMU binary after installation: $qemu_path"
                break
            fi
        done
    fi
fi

if [ -n "$QEMU_BINARY" ]; then
    # Ensure target directory exists
    mkdir -p "$TEMP_DIR/usr/bin"
    cp "$QEMU_BINARY" "$TEMP_DIR/usr/bin/"
    echo_stamp "Copied QEMU binary to chroot"
else
    echo_stamp "Error: No QEMU binary found for ARM emulation - build will fail"
    exit 1
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
        CHROOT_SCRIPT_PATH="/builder/$SCRIPT_NAME"
        
        if [ ! -f "$TEMP_DIR$CHROOT_SCRIPT_PATH" ]; then
            echo_stamp "Copying script $SCRIPT to chroot as $CHROOT_SCRIPT_PATH"
            # Create directory structure if it doesn't exist
            mkdir -p "$(dirname "$TEMP_DIR$CHROOT_SCRIPT_PATH")"
            cp "$SCRIPT" "$TEMP_DIR$CHROOT_SCRIPT_PATH"
            chmod +x "$TEMP_DIR$CHROOT_SCRIPT_PATH"
        fi
        
        # Execute script in chroot using QEMU directly
        echo_stamp "Executing script in chroot: $CHROOT_SCRIPT_PATH"
        
        # Try to use QEMU directly if available
        if [ -n "$QEMU_BINARY" ]; then
            echo_stamp "Using QEMU for execution: $QEMU_BINARY"
            "$QEMU_BINARY" "$TEMP_DIR$CHROOT_SCRIPT_PATH" "$@"
        else
            # Fallback to chroot (may not work in Docker)
            echo_stamp "Using chroot for execution (fallback)"
            chroot "$TEMP_DIR" /bin/sh "$CHROOT_SCRIPT_PATH" "$@"
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
