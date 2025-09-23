#!/bin/bash
# Create meta package for easy installation

set -e

echo "Creating meta package for Coptra..."

# Create meta package directory structure
mkdir -p coptra-meta/debian
mkdir -p coptra-meta/scripts

# Copy debian files
cp debian/control coptra-meta/debian/
cp debian/rules coptra-meta/debian/
cp debian/coptra-meta.install coptra-meta/debian/

# Make rules executable
chmod +x coptra-meta/debian/rules

# Copy configuration and scripts
cp -r config coptra-meta/
cp -r generated_launch coptra-meta/
cp scripts/config_manager.py coptra-meta/scripts/
cp scripts/setup_config.sh coptra-meta/scripts/

# Create changelog
cat > coptra-meta/debian/changelog << 'EOF'
coptra-meta (0.1.0-1) focal; urgency=medium

  * Initial release of Coptra meta package
  * Includes all Coptra components and dependencies
  * Provides easy installation and configuration

 -- Package Maintainer <maintainer@example.com>  $(date -R)
EOF

# Create postinst script
cat > coptra-meta/debian/postinst << 'EOF'
#!/bin/bash
set -e

case "$1" in
    configure)
        # Enable systemd service
        systemctl daemon-reload
        systemctl enable coptra.service
        
        # Set up configuration
        if [ ! -f /etc/coptra/coptra_config.yaml ]; then
            cp /etc/coptra/coptra_config.yaml /etc/coptra/coptra_config.yaml.backup
        fi
        
        echo "Coptra meta package installed successfully!"
        echo "Edit /etc/coptra/coptra_config.yaml to customize settings"
        echo "Run 'systemctl start coptra' to start the service"
        ;;
esac

exit 0
EOF

chmod +x coptra-meta/debian/postinst

# Create prerm script
cat > coptra-meta/debian/prerm << 'EOF'
#!/bin/bash
set -e

case "$1" in
    remove)
        # Stop and disable service
        systemctl stop coptra.service || true
        systemctl disable coptra.service || true
        ;;
esac

exit 0
EOF

chmod +x coptra-meta/debian/prerm

echo "Meta package created in coptra-meta/ directory"
echo "To build: cd coptra-meta && dpkg-buildpackage -us -uc"
