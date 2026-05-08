#!/bin/bash
# ============================================================
# Setup script for Mars Landing Simulation
# Run once after cloning the repo
# ============================================================
set -e

echo "=== Mars Landing Simulation Setup ==="

# 1. ROS2 packages
echo "[1/5] Installing ROS2 packages..."
sudo apt update -q
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-robot-localization \
    ros-jazzy-cv-bridge \
    ros-jazzy-launch-ros \
    python3-cv-bridge

# 2. Python packages
echo "[2/5] Installing Python packages..."
pip3 install numpy opencv-python --break-system-packages

# 3. Optional: MiDaS monocular depth
read -p "Install PyTorch for monocular depth estimation? (y/N): " yn
if [[ "$yn" =~ ^[Yy]$ ]]; then
    pip3 install torch torchvision --break-system-packages
fi

# 4. Copy models to Gazebo
echo "[3/5] Copying models to ~/.gz/models/..."
mkdir -p ~/.gz/models
cp -r models/lander ~/.gz/models/
cp -r models/mars_terrain ~/.gz/models/

# 5. Set environment variables
echo "[4/5] Setting environment variables..."
if ! grep -q "GZ_SIM_RESOURCE_PATH" ~/.bashrc; then
    echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models' >> ~/.bashrc
    echo "Added GZ_SIM_RESOURCE_PATH to ~/.bashrc"
fi

# 6. Create mars_sim shortcut
echo "[5/5] Creating mars_sim shortcut..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

if ! grep -q "mars_sim()" ~/.bashrc; then
cat >> ~/.bashrc << BASHEOF

# Mars Landing Simulation shortcut
mars_sim() {
    rm -rf ~/.gz/sim/*/ogre
    export GZ_SIM_RESOURCE_PATH=~/.gz/models
    cd $REPO_DIR
    gz sim -s worlds/mars_terrain.world --render-engine ogre -v 4 &
    sleep 5
    gz sim -g --render-engine ogre
}
BASHEOF
    echo "Added mars_sim() function to ~/.bashrc"
fi

source ~/.bashrc
echo ""
echo "=== Setup complete! ==="
echo "Run: source ~/.bashrc"
echo "Then: ./scripts/run_simulation.sh"
