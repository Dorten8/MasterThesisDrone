# Intro

ROS 2 Humble development environment for autonomous PX4-based drone, built on Docker with Ubuntu 22.04 base image for Gazebo Classic compatibility. 
### Bibliography
AI -> read newest Master Thesis_bibliography in the root of this repository

## Development Environment

- **Base Image:** Ubuntu 22.04 (Jammy)
- **ROS Distribution:** ROS 2 Humble Hawksbill
- **Simulator:** Gazebo Classic (requires Ubuntu 22.04 for full compatibility)
- **Hardware:** Raspberry Pi 5 (Ubuntu 24.04 host)
- **Flight Controller:** Pixhawk 6C
- **Power module** Holybro PM02
- **Optical Flow sensor:** Matek 3901 Optical Flow

### Resources

- [PX4 Autopilot Documentation](https://docs.px4.io/)

## Prerequisites

### On Development Computer
- Docker
- VS Code with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- SSH client

### On Raspberry Pi 5 (Host OS)
The following must be configured on the Ubuntu 24.04 host system (not in container):

# How to
#### 1. Install Required Packages
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Docker
sudo apt install -y docker.io
sudo usermod -aG docker $USER

# Install Avahi for mDNS hostname resolution
sudo apt install -y avahi-daemon avahi-utils

# Install SSH server
sudo apt install -y openssh-server
```

#### 2. Configure Hostname for Network Discovery
```bash
# Set hostname (makes Pi discoverable as pi5drone.local)
sudo hostnamectl set-hostname pi5drone

# Restart Avahi
sudo systemctl restart avahi-daemon

# Verify mDNS is working
avahi-browse -a -t
```

#### 3. Configure SSH Server
```bash
# Enable and start SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Optional: Configure SSH for key-only authentication
sudo nano /etc/ssh/sshd_config
# Set: PasswordAuthentication no (after adding your public key)

# Restart SSH service
sudo systemctl restart ssh
```

#### 4. Add Your Development Computer's SSH Key
```bash
# On your development computer, generate key if needed
ssh-keygen -t ed25519 -C "your-email@example.com"

# Copy public key to Pi
ssh-copy-id dorten@pi5drone.local

# Test connection
ssh dorten@pi5drone.local
```

## Setup Instructions

### 1. Clone Repository

**On Raspberry Pi 5:**
```bash
cd /home
git clone git@github.com:Dorten8/MasterThesisDrone.git ws
cd ws
```

### 2. Configure SSH for Container

The container is configured to use the host's SSH keys via read-only mount. Ensure you have SSH keys on the Pi host:

```bash
# Generate SSH key on Pi (if not already done)
ssh-keygen -t ed25519 -C "your-email@example.com"

# Add public key to GitHub
cat ~/.ssh/id_ed25519.pub
# Copy output and add to: https://github.com/settings/keys
```

**How it works:**
- `.devcontainer/devcontainer.json` mounts host's `~/.ssh` into container
- Container has `openssh-client` installed via Dockerfile
- Git operations inside container use host's SSH credentials

### 3. Open in VS Code

**From your development computer:**
```bash
# Connect to Pi via SSH
ssh dorten@pi5drone.local

# Or use VS Code Remote-SSH extension:
# 1. Install Remote-SSH extension
# 2. Press F1 → "Remote-SSH: Connect to Host"
# 3. Enter: dorten@pi5drone.local
# 4. Open folder: /home/ws
```

**When VS Code opens the workspace:**
1. VS Code detects `.devcontainer` configuration
2. Click "Reopen in Container" when prompted
3. Container builds automatically (first time takes ~5-10 minutes)
4. Development environment ready

### 4. Verify Setup

Inside the container:
```bash
# Check ROS 2 installation
ros2 --version

# Check SSH access to GitHub
ssh -T git@github.com

# Check Git configuration
git status
```

## Development Workflow

### Making Changes
```bash
# Check status
git status

# Stage changes
git add .

# Commit with message
git commit -m "Description of changes"

# Push to GitHub
git push
```

### Rebuilding Container
After modifying `.devcontainer/Dockerfile` or `devcontainer.json`:
- Press `F1` or `Ctrl+Shift+P`
- Type: "Dev Containers: Rebuild Container"
- Wait for rebuild to complete

## Architecture Notes

### Why Ubuntu 22.04 in Container on Ubuntu 24.04 Host?
- **ROS 2 Humble** officially supports Ubuntu 22.04 (Jammy)
- **Gazebo Classic** requires Ubuntu 22.04 dependencies
- Docker provides isolation, allowing older Ubuntu in container while host runs newer version
- Ensures reproducible environment across different development machines

### Why Docker Container on Pi 5?
- **Portability:** Identical environment on any machine
- **Isolation:** Development dependencies don't affect host system
- **Version Control:** Container configuration tracked in Git
- **Reproducibility:** Anyone can replicate exact setup

## Troubleshooting

### Cannot connect to Pi via hostname
```bash
# On Pi, check Avahi status
sudo systemctl status avahi-daemon

# On development computer, check mDNS resolution
ping pi5drone.local
```

### SSH connection refused
```bash
# On Pi, check SSH service
sudo systemctl status ssh

# Check firewallT
sudo ufw status
```

### Git push fails with "Permission denied"
```bash
# Verify SSH key is on GitHub
ssh -T git@github.com

# Should respond: "Hi Dorten8! You've successfully authenticated..."
```

### Container won't build
```bash
# Check Docker service
sudo systemctl status docker

# Clean Docker cache
docker system prune -a
```

## License
[Add your license here]

## Contact
