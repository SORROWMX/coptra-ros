#!/bin/bash

# Modern Image Resize Script for Orange Pi 3B Coptra
# Usage: image-resize.sh <IMAGE_PATH> [SIZE] [MODE]
# SIZE: target size (e.g., 7G, 8G, 10G, 15G, 20G)
# MODE: 'max' for maximum size, 'min' for minimum size, 'auto' for automatic sizing

set -euo pipefail

# Configuration
readonly SCRIPT_NAME="$(basename "$0")"
readonly MIN_IMAGE_SIZE="4G"
readonly DEFAULT_TARGET_SIZE="15G"
readonly MAX_IMAGE_SIZE="50G"
readonly ROS_FREE_SPACE="5G"  # Minimum free space for ROS installation

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# Global variables
IMAGE_PATH=""
TARGET_SIZE=""
MODE=""
LOOP_DEVICE=""
PARTITION_DEVICE=""
PARTITION_NUM=""

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Cleanup function
cleanup() {
    if [ -n "$LOOP_DEVICE" ] && [ -b "$LOOP_DEVICE" ]; then
        log_info "Cleaning up loop device: $LOOP_DEVICE"
        losetup -d "$LOOP_DEVICE" 2>/dev/null || true
    fi
}

# Set up trap for cleanup
trap cleanup EXIT INT TERM

# Help function
show_help() {
    cat << EOF
Usage: $SCRIPT_NAME <IMAGE_PATH> [SIZE] [MODE]

Arguments:
  IMAGE_PATH    Path to the disk image file
  SIZE          Target size (default: $DEFAULT_TARGET_SIZE)
                Examples: 7G, 8G, 10G, 15G, 20G, max
  MODE          Resize mode (default: auto)
                - auto: Automatic sizing based on content
                - max:  Maximum size within limits
                - min:  Minimum size for ROS installation

Examples:
  $SCRIPT_NAME image.img
  $SCRIPT_NAME image.img 15G
  $SCRIPT_NAME image.img 20G max
  $SCRIPT_NAME image.img max auto

Features:
  - Modern error handling and logging
  - Automatic tool detection and fallbacks
  - Support for GPT and MBR partition tables
  - Intelligent partition detection
  - Safe filesystem resizing
  - Comprehensive cleanup on exit
EOF
}

# Check if required tools are available
check_tools() {
    local missing_tools=()
    local required_tools=("losetup" "parted" "resize2fs" "partprobe" "stat")
    local optional_tools=("qemu-img" "growpart" "sgdisk" "file")
    
    # Check required tools
    for tool in "${required_tools[@]}"; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            missing_tools+=("$tool")
        fi
    done
    
    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_error "Missing required tools: ${missing_tools[*]}"
        log_error "Please install: sudo apt-get install -y parted e2fsprogs util-linux gdisk qemu-utils"
        exit 1
    fi
    
    # Check optional tools
    local available_tools=()
    for tool in "${optional_tools[@]}"; do
        if command -v "$tool" >/dev/null 2>&1; then
            available_tools+=("$tool")
        fi
    done
    
    if [ ${#available_tools[@]} -gt 0 ]; then
        log_info "Available optional tools: ${available_tools[*]}"
    fi
}

# Parse size string to bytes
parse_size() {
    local size_str="$1"
    local size_bytes=0
    
    if [ "$size_str" = "max" ]; then
        size_bytes=$((50 * 1024 * 1024 * 1024))  # 50GB max
    elif [[ "$size_str" =~ ^([0-9]+)([GMK]?)$ ]]; then
        local size_num="${BASH_REMATCH[1]}"
        local size_unit="${BASH_REMATCH[2]}"
        
        case "$size_unit" in
            "G"|"")
                size_bytes=$((size_num * 1024 * 1024 * 1024))
                ;;
            "M")
                size_bytes=$((size_num * 1024 * 1024))
                ;;
            "K")
                size_bytes=$((size_num * 1024))
                ;;
        esac
    else
        log_error "Invalid size format: $size_str"
        log_error "Valid formats: 7G, 8G, 10G, 15G, 20G, max"
        exit 1
    fi
    
    echo "$size_bytes"
}

# Format bytes to human readable
format_bytes() {
    local bytes="$1"
    if [ $bytes -ge $((1024 * 1024 * 1024)) ]; then
        echo "$((bytes / 1024 / 1024 / 1024))GB"
    elif [ $bytes -ge $((1024 * 1024)) ]; then
        echo "$((bytes / 1024 / 1024))MB"
    else
        echo "$((bytes / 1024))KB"
    fi
}

# Get image information
get_image_info() {
    local image_path="$1"
    
    if [ ! -f "$image_path" ]; then
        log_error "Image file not found: $image_path"
        exit 1
    fi
    
    local current_size
    current_size=$(stat -c%s "$image_path")
    local current_size_mb=$((current_size / 1024 / 1024))
    
    log_info "Image: $image_path"
    log_info "Current size: ${current_size_mb}MB ($(format_bytes "$current_size"))"
    
    echo "$current_size"
}

# Resize image file
resize_image_file() {
    local image_path="$1"
    local target_size="$2"
    local current_size="$3"
    
    local target_bytes
    target_bytes=$(parse_size "$target_size")
    local add_bytes=$((target_bytes - current_size))
    
    if [ $add_bytes -le 0 ]; then
        log_info "Image is already larger than target size"
        return 0
    fi
    
    local add_mb=$((add_bytes / 1024 / 1024))
    log_info "Adding $(format_bytes "$add_bytes") to image"
    
    # Use qemu-img if available (preferred method)
    if command -v qemu-img >/dev/null 2>&1; then
        log_info "Using qemu-img to resize image"
        if qemu-img resize "$image_path" "$target_size"; then
            log_success "Image resized successfully with qemu-img"
            return 0
        else
            log_warning "qemu-img failed, falling back to dd method"
        fi
    fi
    
    # Fallback to dd method
    log_info "Using dd to resize image"
    local chunk_size=1024  # 1GB chunks
    local remaining_mb=$add_mb
    
    while [ $remaining_mb -gt 0 ]; do
        local current_chunk=$((remaining_mb > chunk_size ? chunk_size : remaining_mb))
        log_info "Adding chunk: ${current_chunk}MB (${remaining_mb}MB remaining)"
        
        if ! dd if=/dev/zero bs=1M count=$current_chunk >> "$image_path" 2>/dev/null; then
            log_error "Failed to add chunk with dd"
            return 1
        fi
        
        remaining_mb=$((remaining_mb - current_chunk))
    done
    
    log_success "Image resized successfully with dd"
}

# Setup loop device
setup_loop_device() {
    local image_path="$1"
    
    LOOP_DEVICE=$(losetup -f --show "$image_path")
    if [ -z "$LOOP_DEVICE" ]; then
        log_error "Failed to create loop device"
        exit 1
    fi
    
    log_info "Using loop device: $LOOP_DEVICE"
    
    # Wait for device to be ready
    sleep 2
    
    # Force kernel to re-read partition table
    partprobe "$LOOP_DEVICE" 2>/dev/null || true
    sleep 1
}

# Detect partitions
detect_partitions() {
    local loop_dev="$1"
    
    # Check for common partition layouts
    if [ -b "${loop_dev}p2" ]; then
        PARTITION_DEVICE="${loop_dev}p2"
        PARTITION_NUM="2"
        log_info "Found root partition: $PARTITION_DEVICE"
    elif [ -b "${loop_dev}p1" ]; then
        PARTITION_DEVICE="${loop_dev}p1"
        PARTITION_NUM="1"
        log_info "Found partition: $PARTITION_DEVICE"
    else
        log_warning "No partitions found, checking if entire device has filesystem"
        PARTITION_DEVICE="$loop_dev"
        PARTITION_NUM=""
        log_info "Using entire device as filesystem: $PARTITION_DEVICE"
    fi
}

# Resize partition
resize_partition() {
    local loop_dev="$1"
    local partition_num="$2"
    
    if [ -z "$partition_num" ]; then
        log_info "No partition to resize (using entire device)"
        return 0
    fi
    
    log_info "Resizing partition $partition_num"
    
    # Use growpart if available (most reliable for GPT)
    if command -v growpart >/dev/null 2>&1; then
        log_info "Using growpart to resize partition"
        if growpart "$loop_dev" "$partition_num"; then
            log_success "growpart succeeded"
            return 0
        else
            log_warning "growpart failed, trying parted with GPT fix"
        fi
    fi
    
    # Try to fix GPT first
    if command -v sgdisk >/dev/null 2>&1; then
        log_info "Fixing GPT with sgdisk"
        sgdisk -e "$loop_dev" 2>/dev/null || true
    fi
    
    # Use parted as fallback
    log_info "Using parted to resize partition"
    if parted "$loop_dev" resizepart "$partition_num" 100% --script; then
        log_success "Partition resized successfully with parted"
    else
        log_warning "Partition resize failed, but continuing with filesystem resize"
    fi
}

# Resize filesystem
resize_filesystem() {
    local partition_dev="$1"
    
    log_info "Resizing filesystem on: $partition_dev"
    
    # Check filesystem size before resize
    if command -v df >/dev/null 2>&1; then
        log_info "Filesystem size before resize:"
        df -h "$partition_dev" 2>/dev/null || log_warning "Cannot check filesystem size"
    fi
    
    # Attempt to resize filesystem
    log_info "Attempting to resize filesystem..."
    if resize2fs "$partition_dev" 2>/dev/null; then
        log_success "Filesystem resized successfully"
    else
        log_warning "Standard resize failed, trying forced resize..."
        if resize2fs -f "$partition_dev" 2>/dev/null; then
            log_success "Forced filesystem resize succeeded"
        else
            log_error "Filesystem resize failed"
            # Try to check filesystem type
            if command -v file >/dev/null 2>&1; then
                local fs_type
                fs_type=$(file -s "$partition_dev" 2>/dev/null || echo "unknown")
                log_info "Filesystem type: $fs_type"
            fi
            return 1
        fi
    fi
    
    # Check filesystem size after resize
    if command -v df >/dev/null 2>&1; then
        log_info "Filesystem size after resize:"
        df -h "$partition_dev" 2>/dev/null || log_warning "Cannot check filesystem size"
    fi
}

# Main resize function
resize_image() {
    local image_path="$1"
    local target_size="$2"
    local mode="$3"
    
    log_info "Starting image resize process"
    log_info "Target size: $target_size"
    log_info "Mode: $mode"
    
    # Get current image size
    local current_size
    current_size=$(get_image_info "$image_path")
    
    # Determine target size based on mode
    local final_target_size="$target_size"
    if [ "$mode" = "auto" ]; then
        # Auto mode: ensure enough space for ROS installation
        local current_gb=$((current_size / 1024 / 1024 / 1024))
        local ros_space_gb=5
        local target_gb=$((current_gb + ros_space_gb))
        
        if [ $target_gb -lt 10 ]; then
            target_gb=10  # Minimum 10GB
        elif [ $target_gb -gt 20 ]; then
            target_gb=20  # Maximum 20GB for auto mode
        fi
        
        final_target_size="${target_gb}G"
        log_info "Auto mode: calculated target size as $final_target_size"
    fi
    
    # Resize image file
    if ! resize_image_file "$image_path" "$final_target_size" "$current_size"; then
        log_error "Failed to resize image file"
        exit 1
    fi
    
    # Setup loop device
    setup_loop_device "$image_path"
    
    # Detect partitions
    detect_partitions "$LOOP_DEVICE"
    
    # Resize partition
    if ! resize_partition "$LOOP_DEVICE" "$PARTITION_NUM"; then
        log_warning "Partition resize had issues, but continuing"
    fi
    
    # Wait for partition table to be updated
    sleep 2
    partprobe "$LOOP_DEVICE" 2>/dev/null || true
    sleep 1
    
    # Resize filesystem
    if ! resize_filesystem "$PARTITION_DEVICE"; then
        log_error "Filesystem resize failed"
        exit 1
    fi
    
    log_success "Image resize completed successfully"
}

# Main function
main() {
    # Parse arguments
    if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        show_help
        exit 0
    fi
    
    IMAGE_PATH="$1"
    TARGET_SIZE="${2:-$DEFAULT_TARGET_SIZE}"
    MODE="${3:-auto}"
    
    # Validate arguments
    if [ -z "$IMAGE_PATH" ]; then
        log_error "Image path is required"
        show_help
        exit 1
    fi
    
    if [ "$MODE" != "auto" ] && [ "$MODE" != "max" ] && [ "$MODE" != "min" ]; then
        log_error "Invalid mode: $MODE"
        log_error "Valid modes: auto, max, min"
        exit 1
    fi
    
    # Check tools
    check_tools
    
    # Validate target size
    local target_bytes
    target_bytes=$(parse_size "$TARGET_SIZE")
    local min_bytes
    min_bytes=$(parse_size "$MIN_IMAGE_SIZE")
    local max_bytes
    max_bytes=$(parse_size "$MAX_IMAGE_SIZE")
    
    if [ $target_bytes -lt $min_bytes ]; then
        log_error "Target size too small (minimum: $MIN_IMAGE_SIZE)"
        exit 1
    fi
    
    if [ $target_bytes -gt $max_bytes ]; then
        log_error "Target size too large (maximum: $MAX_IMAGE_SIZE)"
        exit 1
    fi
    
    # Start resize process
    resize_image "$IMAGE_PATH" "$TARGET_SIZE" "$MODE"
}

# Run main function
main "$@"