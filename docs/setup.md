# Setup Guide

## System Requirements

- Ubuntu 22.04 or 24.04
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim 8.x)
- NVIDIA GPU (RTX recommended)
- Python 3.10+

## Known Issues

### Ogre2 crash on Ubuntu 24.04 + NVIDIA driver 590+
Ogre2 crashes with `Fragment Program 700000000PixelShader_ps failed to compile`.
**Workaround**: always use `--render-engine ogre` flag.
This is a known bug in `gz_ogre_next_vendor` bundled with ROS Jazzy.

### Depth camera requires Ogre2
Since Ogre2 is broken, depth camera is unavailable.
**Workaround**: use altimeter (ray sensor) + monocular camera instead.

## Step-by-step Installation

```bash
# 1. Install ROS2 Jazzy
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# 2. Install Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz

# 3. Clone repo
git clone https://github.com/YOUR_USERNAME/mars_landing_ws.git
cd mars_landing_ws

# 4. Run setup
./scripts/setup_env.sh
source ~/.bashrc

# 5. Generate terrain files (run once)
python3 - << 'PYEOF'
from PIL import Image
import numpy as np

# Convert 16-bit Unity heightmap to 8-bit
img = Image.open('models/mars_terrain/materials/textures/terrain.png')
arr = np.array(img, dtype=np.float32)
arr_norm = (arr - arr.min()) / (arr.max() - arr.min()) * 255
Image.fromarray(arr_norm.astype(np.uint8), mode='L').save(
    'models/mars_terrain/materials/textures/terrain_8bit.png')
print('terrain_8bit.png created')
PYEOF

# 6. Copy models
cp -r models/lander ~/.gz/models/
cp -r models/mars_terrain ~/.gz/models/
```

## Running

```bash
# Option 1: Single script (opens 5 terminals)
./scripts/run_simulation.sh

# Option 2: Manual (more control)
# Terminal 1:
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
gz sim -s worlds/mars_terrain.world --render-engine ogre -v 4

# Terminal 2:
gz sim -g --render-engine ogre

# Terminal 3:
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/lander/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /altimeter/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /lander/thrust@geometry_msgs/msg/Wrench]gz.msgs.Wrench

# Terminal 4:
source /opt/ros/jazzy/setup.bash
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml

# Terminal 5:
source /opt/ros/jazzy/setup.bash
python3 src/perception/perception_node.py

# Terminal 6:
source /opt/ros/jazzy/setup.bash
python3 src/guidance/method_C_guidance_node.py
```
