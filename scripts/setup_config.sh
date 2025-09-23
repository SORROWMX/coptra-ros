#!/bin/bash
# Setup script for Coptra configuration system

set -e

echo "Setting up Coptra configuration system..."

# Create necessary directories
mkdir -p config
mkdir -p generated_launch
mkdir -p scripts

# Make scripts executable
chmod +x scripts/config_manager.py

# Initialize default configuration if it doesn't exist
if [ ! -f config/coptra_config.yaml ]; then
    echo "Initializing default configuration..."
    python3 scripts/config_manager.py --init
fi

echo "Configuration system setup complete!"
echo ""
echo "Usage:"
echo "  python3 scripts/config_manager.py --generate  # Generate launch files from config"
echo "  python3 scripts/config_manager.py --init      # Initialize default configuration"
echo ""
echo "Edit config/coptra_config.yaml to customize your settings."
