# Building Orange Pi 3B Image on Virtual Machine

## VM Requirements

### Minimum specifications:
- **OS**: Ubuntu 20.04 LTS or newer
- **RAM**: 4GB (8GB recommended)
- **Disk**: 20GB free space
- **CPU**: 2 cores (4 recommended)

### Install dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y \
    qemu-user-static \
    binfmt-support \
    kpartx \
    parted \
    dosfstools \
    unzip \
    wget \
    curl \
    ca-certificates \
    git \
    build-essential \
    python3 \
    python3-pip

# Setup QEMU for ARM64 emulation
sudo update-binfmts --enable qemu-arm
sudo update-binfmts --enable qemu-aarch64

# Check binfmt configuration
update-binfmts --status
```

## Building the image

### Method 1: Using automated script (Recommended)
```bash
# Clone repository
git clone https://github.com/your-username/coptra.git
cd coptra

# Run automated build script
sudo ./build-on-vm.sh
```

### Method 2: Manual build
```bash
# Clone repository
git clone https://github.com/your-username/coptra.git
cd coptra

# Set environment variables
export DEBIAN_FRONTEND=noninteractive
export TZ=UTC
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# Make scripts executable
chmod +x builder/*.sh

# Run build
sudo ./builder/image-build.sh
```

## Troubleshooting

### Problem: "Permission denied" when mounting
```bash
# Solution: run with sudo
sudo ./builder/image-build.sh
```

### Problem: "No space left on device"
```bash
# Solution: clean cache and temporary files
sudo apt clean
sudo rm -rf /tmp/*
df -h  # check free space
```

### Problem: "QEMU emulation failed"
```bash
# Solution: restart binfmt
sudo systemctl restart systemd-binfmt
sudo update-binfmts --enable qemu-aarch64
```

### Problem: "Loop device busy"
```bash
# Solution: clean loop devices
sudo losetup -D
sudo umount /dev/loop* 2>/dev/null || true
```

### Problem: "Invalid ELF image for this architecture"
```bash
# Solution: ensure binfmt is properly configured
sudo update-binfmts --enable qemu-aarch64
sudo systemctl restart systemd-binfmt
```

## Check results

After successful build, image will be in `images/` folder:
```bash
ls -la images/
# Should see file: coptra_<commit-hash>.img
```

## Compress image (optional)
```bash
# Compress image to save space
xz -9 images/coptra_*.img
```

## Build time

On VM with specifications:
- **4GB RAM, 2 CPU**: ~2-3 hours
- **8GB RAM, 4 CPU**: ~1-2 hours

## Advantages of building on VM

✅ **No Docker issues** - native environment  
✅ **Full control** - all processes visible  
✅ **Easy debugging** - can stop and inspect  
✅ **Fast iteration** - changes apply immediately  

## Disadvantages

❌ **Slower** - ARM64 emulation through QEMU  
❌ **More resources** - need powerful VM  
❌ **Complex setup** - more dependencies  

## Alternative: GitHub Actions

If VM is not suitable, use GitHub Actions:
```bash
# Trigger workflow
git push origin main
# Or via web interface: Actions -> Build Orange Pi 3B Coptra Image
```

## Quick start

1. **Setup VM** with Ubuntu 20.04+ (4GB RAM, 20GB disk)
2. **Install dependencies**: `sudo apt install -y qemu-user-static kpartx parted unzip wget curl git`
3. **Clone repo**: `git clone https://github.com/your-username/coptra.git`
4. **Run build**: `cd coptra && sudo ./build-on-vm.sh`
5. **Wait 1-3 hours** for build to complete
6. **Flash image** to microSD card and test on Orange Pi 3B
