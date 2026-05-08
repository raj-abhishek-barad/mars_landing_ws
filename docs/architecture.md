# System Architecture

## Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         GAZEBO HARMONIC                         │
│                                                                  │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────────┐  │
│  │   LANDER    │    │ MARS TERRAIN │    │     SENSORS       │  │
│  │  2000 kg    │    │  500x500m    │    │                   │  │
│  │  2x2x2m box │    │  Crater      │    │  IMU (100Hz)      │  │
│  │             │    │  heightmap   │    │  Altimeter (50Hz) │  │
│  │  pose(t)    │    │              │    │  Camera (5Hz)     │  │
│  └──────┬──────┘    └──────────────┘    └─────────┬─────────┘  │
│         │  OdometryPublisher                      │             │
│         │  plugin (50Hz)                          │             │
└─────────┼─────────────────────────────────────────┼─────────────┘
          │                                         │
          │         ros_gz_bridge                   │
          ▼                                         ▼
┌─────────────────────┐              ┌──────────────────────────┐
│   /model/lander/    │              │  /imu/data               │
│   odometry          │              │  /altimeter/scan         │
│   nav_msgs/Odometry │              │  /camera/image_raw       │
└──────────┬──────────┘              └─────────────┬────────────┘
           │                                       │
           ├───────────────────────────────────────┤
           │                                       │
           ▼                                       ▼
┌──────────────────────┐          ┌────────────────────────────┐
│   EKF NODE           │          │   PERCEPTION NODE          │
│   (robot_localization│          │                            │
│   50Hz)              │          │  Altimeter callback:       │
│                      │          │   - range → terrain_z      │
│   Fuses:             │          │   - ElevationMap.update()  │
│   + IMU accel/gyro   │          │   - publish altimetry→EKF  │
│   + Altimetry (Z)    │          │                            │
│                      │          │  Camera callback:          │
│   Outputs:           │          │   - brightness gradient    │
│   + position (x,y,z) │          │   - elevation hints        │
│   + velocity         │          │                            │
│   + covariance P     │          │  AdaptiveStepFitter:       │
└──────────┬───────────┘          │   - greedy merge           │
           │                      │   - variable N steps       │
           │ /odometry/filtered   │   - safety margin +50m     │
           │                      └─────────────┬──────────────┘
           │                                    │ /terrain/barriers
           │                                    │ [n,h0..hn,w0..wn]
           └──────────────────┬─────────────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │   METHOD C GUIDANCE NODE      │
              │                               │
              │  @ 10 Hz:                     │
              │  1. Read r, v, P from EKF     │
              │  2. Get h_bar,w_bar from      │
              │     perception (thread-safe)  │
              │  3. Compute:                  │
              │     a = a_ZEM/ZEV             │
              │       + a_barrier(h,w)        │
              │       + a_MSS(S, φ_dynamic)   │
              │  4. Saturate [T_min, T_max]   │
              │  5. Update mass (Isp burn)    │
              │  6. Publish thrust            │
              └──────────────┬────────────────┘
                             │ /lander/thrust
                             │ Wrench [Fx,Fy,Fz]
                             │
                             │ ros_gz_bridge
                             ▼
                    ┌──────────────────┐
                    │  Gazebo          │
                    │  ApplyLinkWrench │
                    │  plugin          │
                    └──────────────────┘
```

## Topic List

| Topic | Type | Direction | Publisher | Subscriber |
|---|---|---|---|---|
| `/model/lander/odometry` | `nav_msgs/Odometry` | Gz→ROS | Gazebo | EKF, Guidance |
| `/imu/data` | `sensor_msgs/Imu` | Gz→ROS | Gazebo | EKF |
| `/altimeter/scan` | `sensor_msgs/LaserScan` | Gz→ROS | Gazebo | Perception |
| `/camera/image_raw` | `sensor_msgs/Image` | Gz→ROS | Gazebo | Perception |
| `/lander/altimetry` | `PoseWithCovarianceStamped` | ROS | Perception | EKF |
| `/terrain/barriers` | `Float64MultiArray` | ROS | Perception | Guidance |
| `/odometry/filtered` | `nav_msgs/Odometry` | ROS | EKF | Guidance, Display |
| `/lander/thrust` | `geometry_msgs/Wrench` | ROS→Gz | Guidance | Gazebo |
| `/lander/state` | `Float64MultiArray` | ROS | Localisation | — |
