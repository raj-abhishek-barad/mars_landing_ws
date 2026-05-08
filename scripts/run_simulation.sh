#!/bin/bash
# ============================================================
# Run the full Mars Landing Simulation
# Opens 5 terminals automatically
# ============================================================

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="$REPO_DIR/worlds/mars_terrain.world"
EKF_CONFIG="$REPO_DIR/config/ekf.yaml"

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models

echo "=== Mars Landing Simulation ==="
echo "Repo: $REPO_DIR"
echo ""

# Clear old cache
rm -rf ~/.gz/sim/*/ogre

# Terminal 1: Gazebo server
echo "[T1] Starting Gazebo server..."
gnome-terminal --title="Gazebo Server" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     export GZ_SIM_RESOURCE_PATH=~/.gz/models && \
     gz sim -s $WORLD --render-engine ogre -v 4; exec bash" &

sleep 3

# Terminal 2: Gazebo GUI
echo "[T2] Starting Gazebo GUI..."
gnome-terminal --title="Gazebo GUI" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     gz sim -g --render-engine ogre; exec bash" &

sleep 2

# Unpause simulation
echo "Unpausing simulation..."
gz service -s /world/mars_world/control \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req 'pause: false' 2>/dev/null || true

# Terminal 3: ROS-Gazebo bridge
echo "[T3] Starting ROS-Gazebo bridge..."
gnome-terminal --title="ROS-Gz Bridge" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 run ros_gz_bridge parameter_bridge \
       /model/lander/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
       /altimeter/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
       /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
       /imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU \
       /lander/thrust@geometry_msgs/msg/Wrench]gz.msgs.Wrench; exec bash" &

sleep 2

# Terminal 4: EKF + Perception + Localisation
echo "[T4] Starting EKF and perception..."
gnome-terminal --title="EKF + Perception" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 run robot_localization ekf_node \
       --ros-args --params-file $EKF_CONFIG &
     sleep 2 && \
     python3 $REPO_DIR/src/perception/perception_node.py &
     python3 $REPO_DIR/src/localisation/lander_localisation.py; exec bash" &

sleep 3

# Terminal 5: Method C Guidance
echo "[T5] Starting Method C guidance..."
gnome-terminal --title="Method C Guidance" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     python3 $REPO_DIR/src/guidance/method_C_guidance_node.py; exec bash" &

echo ""
echo "=== All terminals launched ==="
echo "Watch Terminal 5 for guidance output."
echo "Watch Terminal 4 for localisation and terrain map updates."
