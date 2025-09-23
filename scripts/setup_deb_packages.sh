#!/bin/bash
# Setup script for building .deb packages with bloom

set -e

echo "Setting up .deb package building with bloom..."

# Install bloom if not already installed
if ! command -v bloom-release &> /dev/null; then
    echo "Installing bloom..."
    sudo apt-get update
    sudo apt-get install -y python-bloom
fi

# Install fakeroot for building packages
sudo apt-get install -y fakeroot

# Create bloom workspace
mkdir -p bloom_workspace
cd bloom_workspace

# Initialize bloom workspace
bloom-config new workspace

echo "Bloom workspace initialized!"
echo ""
echo "To build packages:"
echo "1. Configure your release repositories in bloom_config/*.yaml"
echo "2. Run: ./scripts/build_packages.sh"
echo ""
echo "Make sure to update the repository URLs in bloom_config/*.yaml files"
