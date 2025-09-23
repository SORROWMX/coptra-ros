#!/bin/bash
# Build .deb packages using bloom

set -e

PACKAGES=("coptra" "aruco_pose" "coptra_blocks" "roswww_static")
BLOOM_CONFIG_DIR="bloom_config"
OUTPUT_DIR="deb_packages"

echo "Building .deb packages with bloom..."

# Create output directory
mkdir -p $OUTPUT_DIR

# Build each package
for package in "${PACKAGES[@]}"; do
    echo "Building package: $package"
    
    # Check if config exists
    if [ ! -f "$BLOOM_CONFIG_DIR/${package}.yaml" ]; then
        echo "Warning: No bloom config found for $package, skipping..."
        continue
    fi
    
    # Create temporary directory for this package
    TEMP_DIR="temp_${package}"
    mkdir -p $TEMP_DIR
    cd $TEMP_DIR
    
    # Initialize bloom release
    bloom-release --rosdistro noetic --track noetic $package \
        --edit-track \
        --config-file "../$BLOOM_CONFIG_DIR/${package}.yaml"
    
    # Build the package
    bloom-release --rosdistro noetic --track noetic $package \
        --config-file "../$BLOOM_CONFIG_DIR/${package}.yaml" \
        --debian-branch main \
        --debian-distro focal
    
    # Copy built packages to output directory
    find . -name "*.deb" -exec cp {} "../$OUTPUT_DIR/" \;
    
    # Cleanup
    cd ..
    rm -rf $TEMP_DIR
    
    echo "Package $package built successfully!"
done

echo "All packages built successfully!"
echo "Find .deb files in: $OUTPUT_DIR/"
