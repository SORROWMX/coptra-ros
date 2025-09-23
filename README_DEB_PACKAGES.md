# Coptra .deb Packages

This document describes how to build and install Coptra as .deb packages.

## Overview

The Coptra project is split into several ROS packages that can be built as individual .deb packages:

- `coptra` - Main drone control package
- `aruco-pose` - ArUco marker positioning
- `coptra-blocks` - Blockly programming interface
- `roswww-static` - Static web content
- `coptra-meta` - Meta package for easy installation

## Prerequisites

### System Requirements
- Ubuntu 20.04 (Focal) or compatible
- ROS Noetic
- Python 3.8+

### Dependencies Installation

```bash
# Install ROS Noetic
sudo apt-get install -y ros-noetic-desktop-full

# Install build dependencies
sudo apt-get install -y python-bloom fakeroot debhelper

# Install Orange Pi ROS repository
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://sorrowmx.github.io/orangepi3b-ros-noetic/public-key.asc | sudo gpg --dearmor -o /etc/apt/keyrings/orangepi-ros-noetic.gpg
sudo chmod 0644 /etc/apt/keyrings/orangepi-ros-noetic.gpg
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/orangepi-ros-noetic.gpg] https://sorrowmx.github.io/orangepi3b-ros-noetic/debian bookworm main" | sudo tee /etc/apt/sources.list.d/ros-noetic-orangepi.list
sudo apt update
```

## Configuration System

### Easy Configuration Management

The project includes a configuration system that allows easy customization of launch files:

```bash
# Initialize configuration system
./scripts/setup_config.sh

# Edit configuration
nano config/coptra_config.yaml

# Generate launch files from configuration
python3 scripts/config_manager.py --generate
```

### Configuration Options

The main configuration file `config/coptra_config.yaml` includes:

- **MAVROS settings**: Connection type, IP, system ID
- **Camera settings**: Direction, device, throttling
- **ArUco settings**: Detection, mapping, placement
- **LED settings**: Strip configuration, effects
- **Component toggles**: Enable/disable features

## Building Packages

### Using Scripts

```bash
# Setup build environment
./scripts/setup_deb_packages.sh

# Build all packages
./scripts/build_packages.sh

# Create meta package
./scripts/create_meta_package.sh
```

### Manual Building

```bash
# For each package
cd bloom_workspace
bloom-release --rosdistro noetic --track noetic <package_name>
```

## Installation

### Individual Packages

```bash
# Install individual packages
sudo dpkg -i coptra_0.1.0-1_arm64.deb
sudo dpkg -i aruco-pose_0.25.0-1_arm64.deb
sudo dpkg -i coptra-blocks_0.25.0-1_arm64.deb
sudo dpkg -i roswww-static_0.25.0-1_arm64.deb
```

### Meta Package (Recommended)

```bash
# Install meta package (installs all dependencies)
sudo dpkg -i coptra-meta_0.1.0-1_arm64.deb
```

## Configuration After Installation

### System Configuration

```bash
# Edit system configuration
sudo nano /etc/coptra/coptra_config.yaml

# Regenerate launch files
sudo python3 /opt/ros/noetic/share/coptra/scripts/config_manager.py --generate

# Start service
sudo systemctl start coptra
sudo systemctl enable coptra
```

### Service Management

```bash
# Start/stop service
sudo systemctl start coptra
sudo systemctl stop coptra
sudo systemctl restart coptra

# Check status
sudo systemctl status coptra

# View logs
journalctl -u coptra -f
```

## GitHub Actions

The project includes GitHub Actions workflow for automated building:

- Triggers on version tags (v*)
- Builds packages for Ubuntu 20.04 (Focal)
- Creates GitHub releases with .deb files
- Supports ARM64 architecture

## Troubleshooting

### Common Issues

1. **Missing dependencies**: Run `./scripts/install_dependencies.sh`
2. **Configuration errors**: Check `/etc/coptra/coptra_config.yaml`
3. **Service won't start**: Check logs with `journalctl -u coptra`

### Debug Mode

```bash
# Run manually for debugging
roslaunch coptra coptra_generated.launch
```

## File Structure

```
├── config/
│   └── coptra_config.yaml          # Main configuration
├── scripts/
│   ├── config_manager.py           # Configuration manager
│   ├── setup_config.sh            # Setup script
│   ├── build_packages.sh          # Build script
│   └── install_dependencies.sh    # Dependencies installer
├── bloom_config/                  # Bloom configurations
├── generated_launch/              # Generated launch files
├── deb_packages/                  # Built .deb files
└── .github/workflows/             # GitHub Actions
```

## Contributing

1. Edit configuration in `config/coptra_config.yaml`
2. Test with `python3 scripts/config_manager.py --generate`
3. Build packages with `./scripts/build_packages.sh`
4. Test installation on target system
