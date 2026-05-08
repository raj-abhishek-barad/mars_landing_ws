# Mars Soft Landing Simulation

A ROS2 + Gazebo Harmonic simulation of Mars powered descent guidance using **ROBUST CLOSING VELOCITY PENALTY TERM** — a robust terrain-aware guidance law combining ZEM/ZEV optimal guidance, barrier penalty functions, and a Modified Super-Twisting Sliding Mode (MSS) robust term.

## Features

- **Method C Guidance** — ZEM/ZEV + adaptive terrain barrier penalty + MSS disturbance rejection
- **Adaptive Terrain Mapping** — builds elevation map on-the-go from altimeter + monocular camera
- **Uncertainty-Aware Control** — boundary layer dynamically scales with EKF position covariance
- **No GPS** — localisation from IMU + altimeter only
- **Mars terrain** from Unity-exported heightmap (8-bit grayscale PNG)

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    GAZEBO SIM                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │  Lander  │  │  Terrain │  │  Sensors         │  │
│  │  (SDF)   │  │  (mesh)  │  │  IMU/Altimeter   │  │
│  │  2000kg  │  │  500x500 │  │  Monocular Cam   │  │
│  └──────────┘  └──────────┘  └──────────────────┘  │
└────────────┬──────────────────────────┬─────────────┘
             │ ros_gz_bridge            │
    ┌────────▼────────┐      ┌──────────▼──────────┐
    │  Perception     │      │   Localisation      │
    │  Node           │      │   (EKF)             │
    │  - Altimeter    │      │   robot_localization│
    │  - Camera       │      │   IMU + Altimeter   │
    │  - ElevMap      │      └──────────┬──────────┘
    │  - StepFitter   │                 │ /odometry/filtered
    └────────┬────────┘                 │
             │ /terrain/barriers        │
    ┌────────▼──────────────────────────▼──────────┐
    │           Method C Guidance Node             │
    │   ZEM/ZEV + Barrier Penalty + MSS Robust     │
    │   Thrust saturation [6200N, 24800N]          │
    │   Isp=225s, m0=2000kg                        │
    └────────────────────┬─────────────────────────┘
                         │ /lander/thrust (Wrench)
                         │ ros_gz_bridge
                         ▼
                    Gazebo (force applied)
```

## Package Structure

```
mars_landing_ws/
├── README.md
├── requirements.txt
├── config/
│   └── ekf.yaml                    # EKF configuration
├── models/
│   ├── lander/
│   │   ├── model.config
│   │   └── model.sdf               # Lander with IMU, altimeter, camera
│   └── mars_terrain/
│       ├── model.config
│       ├── model.sdf               # Terrain mesh model
│       └── materials/
│           ├── textures/
│           │   ├── terrain_8bit.png    # Heightmap (8-bit grayscale)
│           │   └── mars_texture.png    # Surface color texture
│           └── scripts/
│               └── mars.material       # Ogre1 material script
├── worlds/
│   └── mars_terrain.world          # Gazebo world file
├── src/
│   ├── guidance/
│   │   └── method_C_guidance_node.py
│   ├── perception/
│   │   ├── perception_node.py
│   │   └── terrain_mapper_camera.py
│   ├── localisation/
│   │   └── lander_localisation.py
│   └── launch/
│       └── mars_landing.launch.py
├── scripts/
│   ├── setup_env.sh
│   └── run_simulation.sh
└── docs/
    ├── architecture.md
    ├── method_c.md
    └── setup.md
```

## Quick Start

```bash
# 1. Clone
git clone https://github.com/YOUR_USERNAME/mars_landing_ws.git
cd mars_landing_ws

# 2. Install dependencies
./scripts/setup_env.sh

# 3. Copy model files to Gazebo
cp -r models/lander ~/.gz/models/
cp -r models/mars_terrain ~/.gz/models/

# 4. Set Gazebo resource path
echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models' >> ~/.bashrc
source ~/.bashrc

# 5. Run everything
./scripts/run_simulation.sh
```

## Initial Conditions (Case 3)

```
Position : [900, 800, 2200] m
Velocity : [15, 25, -80] m/s
Mass     : 2000 kg
Target   : [0, 0, 0] m at [0, 0, 0] m/s
```

## Dependencies

- ROS2 Jazzy
- Gazebo Harmonic (gz-sim 8.x)
- ros-jazzy-ros-gz-bridge
- ros-jazzy-robot-localization
- Python: numpy, opencv-python, cv-bridge
- Optional: torch, torchvision (for MiDaS monocular depth)

## Known Issues

- Ogre2 renderer crashes on Ubuntu 24.04 with NVIDIA driver 590+ (Mesa/EGL conflict)
- Use `--render-engine ogre` flag as workaround
- Depth camera requires Ogre2 — use altimeter + monocular camera instead

## References

- Blackmore et al., "Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem"
- Simplício et al., "Guidance and Control for Powered Descent with Terrain-Relative Navigation"
