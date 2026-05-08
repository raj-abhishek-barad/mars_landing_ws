"""
Perception Node — Fixed + Improved
=====================================
Inputs:
  /altimeter/scan      (sensor_msgs/LaserScan)  — single-beam downward rangefinder
  /camera/image_raw    (sensor_msgs/Image)       — monocular downward camera

Outputs:
  /lander/altimetry    (PoseWithCovarianceStamped) — altitude + uncertainty for EKF
  /terrain/barriers    (Float64MultiArray)          — adaptive barrier params for guidance

Fixes from original:
  1. altitude = msg.ranges[0]  not msg.ranges (was returning full array)
  2. covariance is 36-element list, not scalar
  3. inf/nan check done correctly on scalar
  4. terrain barrier now outputs proper [n, h0..hn, w0..wn] format
  5. Added camera-based terrain estimation using image brightness gradient
  6. Added safety margin and minimum barrier width
  7. Barrier now uses proper multi-step format matching guidance node

Improvements:
  - ElevationMap builds incrementally from altimeter history
  - AdaptiveStepFitter fits variable steps from altitude profile
  - Camera brightness gradient used as terrain roughness proxy
    (bright = high terrain/crater rim, dark = low terrain/crater floor)
  - Full covariance matrix properly populated
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
from collections import deque


# ============================================================
# CONSTANTS
# ============================================================
SAFETY_MARGIN     = 50.0    # m — added above observed terrain
MIN_BARRIER_WIDTH = 50.0    # m
MAX_STEPS         = 15
MIN_STEPS         = 2
MERGE_TOL         = 30.0    # m — merge steps within this elevation diff
ALTIMETER_NOISE   = 0.1     # m stddev


# ============================================================
# INCREMENTAL ELEVATION MAP
# ============================================================
class ElevationMap:
    """
    Builds a radial elevation profile from altimeter history.
    As lander moves laterally, the altimeter samples different
    radial distances → builds up a map over time.
    """
    def __init__(self, r_max=3000.0, bin_size=20.0):
        self.bin_size  = bin_size
        self.n_bins    = int(r_max / bin_size) + 1
        self.elevation = np.full(self.n_bins, -np.inf)
        self.observed  = np.zeros(self.n_bins, dtype=bool)

    def update(self, lander_xy, terrain_z):
        """
        lander_xy: current (x,y) position of lander
        terrain_z: altitude of terrain directly below = lander_z - range
        """
        r   = np.sqrt(lander_xy[0]**2 + lander_xy[1]**2)
        idx = min(int(r / self.bin_size), self.n_bins - 1)
        if terrain_z > self.elevation[idx]:
            self.elevation[idx] = terrain_z
            self.observed[idx]  = True

    def get_observed(self):
        mask  = self.observed
        radii = np.arange(self.n_bins)[mask] * self.bin_size
        elevs = self.elevation[mask]
        return radii, elevs

    def coverage(self):
        return 100.0 * self.observed.sum() / self.n_bins


# ============================================================
# ADAPTIVE STEP FITTER
# ============================================================
class AdaptiveStepFitter:
    def fit(self, radii, elevations,
            safety_margin=SAFETY_MARGIN,
            merge_tol=MERGE_TOL,
            n_min=MIN_STEPS,
            n_max=MAX_STEPS,
            w_min=MIN_BARRIER_WIDTH):

        if len(radii) < 2:
            # Not enough data — return a wide conservative barrier
            return [500.0, 1500.0], [600.0, 1400.0]

        order    = np.argsort(radii)
        r_s      = radii[order]
        z_s      = elevations[order] + safety_margin

        # Greedy merge
        groups, g_r, g_z = [], [r_s[0]], [z_s[0]]
        for i in range(1, len(r_s)):
            if abs(z_s[i] - np.mean(g_z)) < merge_tol:
                g_r.append(r_s[i]); g_z.append(z_s[i])
            else:
                groups.append((max(g_r), max(g_z)))
                g_r, g_z = [r_s[i]], [z_s[i]]
        groups.append((max(g_r), max(g_z)))

        # Cap
        if len(groups) > n_max:
            idx    = np.round(np.linspace(0, len(groups)-1, n_max)).astype(int)
            groups = [groups[i] for i in idx]
        while len(groups) < n_min:
            groups.append((groups[-1][0]*1.5, groups[-1][1]))

        # Enforce monotone w
        w_bar, h_bar, w_prev = [], [], w_min
        for w, h in groups:
            w = max(w, w_prev + w_min)
            w_bar.append(float(w))
            h_bar.append(float(max(h, 0.0)))
            w_prev = w

        return h_bar, w_bar


# ============================================================
# PERCEPTION NODE
# ============================================================
class RealMissionPerception(Node):
    def __init__(self):
        super().__init__('real_mission_perception')

        self.bridge   = CvBridge()
        self.elev_map = ElevationMap()
        self.fitter   = AdaptiveStepFitter()

        # Current lander state (from odometry)
        self.lander_pos = np.array([0.0, 0.0, 200.0])

        # Current barrier (updated incrementally)
        self.h_bar = [500.0, 1500.0]
        self.w_bar = [600.0, 1400.0]

        # Rolling altitude buffer for smoothing
        self.alt_buffer = deque(maxlen=10)

        # Subscriptions
        self.create_subscription(
            LaserScan, '/altimeter/scan', self.alt_callback, 10)
        self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(
            Odometry, '/model/lander/odometry', self.odom_callback, 10)

        # Publishers
        self.alt_pub     = self.create_publisher(
            PoseWithCovarianceStamped, '/lander/altimetry', 10)
        self.terrain_pub = self.create_publisher(
            Float64MultiArray, '/terrain/barriers', 10)

        # Publish barrier at 5Hz regardless of sensor input
        self.create_timer(0.2, self.publish_barrier)

        self.get_logger().info('Perception node started.')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.lander_pos = np.array([p.x, p.y, p.z])

    def alt_callback(self, msg):
        # FIX 1: ranges is an array — take first (and only) element
        if len(msg.ranges) == 0:
            return
        raw_range = msg.ranges[0]

        # FIX 2: proper scalar inf/nan check
        if not np.isfinite(raw_range) or raw_range > msg.range_max:
            return
        if raw_range < msg.range_min:
            return

        # Smooth with rolling buffer
        self.alt_buffer.append(raw_range)
        altitude = float(np.median(self.alt_buffer))

        # Terrain elevation = lander altitude - range to terrain
        terrain_z = self.lander_pos[2] - altitude

        # Update elevation map with current lateral position
        self.elev_map.update(self.lander_pos[:2], terrain_z)

        # Refit barrier if enough data
        radii, elevs = self.elev_map.get_observed()
        if len(radii) >= 2:
            self.h_bar, self.w_bar = self.fitter.fit(radii, elevs)

        # FIX 3: proper 36-element covariance matrix
        # Altimeter gives good Z, poor XY
        cov = np.zeros(36)
        cov[0]  = 9999.0   # x variance — unknown
        cov[7]  = 9999.0   # y variance — unknown
        cov[14] = ALTIMETER_NOISE**2  # z variance — from altimeter noise
        cov[21] = 9999.0   # roll — unknown
        cov[28] = 9999.0   # pitch — unknown
        cov[35] = 9999.0   # yaw — unknown

        # Publish altimetry for EKF
        p_msg = PoseWithCovarianceStamped()
        p_msg.header          = msg.header
        p_msg.header.frame_id = 'odom'
        p_msg.pose.pose.position.z = self.lander_pos[2] - altitude
        p_msg.pose.covariance  = cov.tolist()   # FIX 3
        self.alt_pub.publish(p_msg)

        self.get_logger().debug(
            f'Altimeter: range={altitude:.1f}m | '
            f'terrain_z={terrain_z:.1f}m | '
            f'map={self.elev_map.coverage():.1f}%'
        )

    def camera_callback(self, msg):
        """
        Use image brightness gradient as terrain roughness proxy.
        Bright regions = high terrain (crater rim lit by sun)
        Dark regions   = low terrain (crater floor in shadow)

        This gives a rough relative elevation hint per image column,
        which we use to refine the radial elevation map.
        """
        try:
            img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'camera_callback: {e}')
            return

        H, W   = img.shape
        alt    = self.lander_pos[2]
        fov    = 1.047  # radians (matches SDF)

        # Ground sample distance at current altitude
        gsd = 2 * alt * np.tan(fov/2) / W  # m/pixel

        # Column-wise mean brightness → relative elevation proxy
        col_brightness = img.mean(axis=0).astype(float)

        # Normalize brightness to elevation hint
        # Brightest column = highest terrain = fraction of altitude
        b_min, b_max = col_brightness.min(), col_brightness.max()
        if b_max - b_min < 5.0:
            return  # uniform image — no useful terrain info

        b_norm = (col_brightness - b_min) / (b_max - b_min)
        # Scale: brightest pixel assumed to be ~20% of altitude above floor
        elev_hint = b_norm * 0.2 * alt

        # Map columns to lateral ground positions
        col_idx   = np.arange(W)
        x_ground  = (col_idx - W/2) * gsd + self.lander_pos[0]
        r_ground  = np.abs(x_ground)

        # Update elevation map with camera hints
        for i in range(0, W, 4):  # subsample every 4 pixels
            r_idx = min(int(r_ground[i] / self.elev_map.bin_size),
                        self.elev_map.n_bins - 1)
            z_hint = float(elev_hint[i])
            if z_hint > self.elev_map.elevation[r_idx]:
                self.elev_map.elevation[r_idx] = z_hint
                self.elev_map.observed[r_idx]  = True

        # Refit after camera update
        radii, elevs = self.elev_map.get_observed()
        if len(radii) >= 2:
            self.h_bar, self.w_bar = self.fitter.fit(radii, elevs)

    def publish_barrier(self):
        """
        Publish barrier as:
        [n_steps, h0, h1, ..., hn, w0, w1, ..., wn, coverage]
        Compatible with method_C_guidance_node_adaptive.py
        """
        n   = len(self.h_bar)
        msg = Float64MultiArray()
        msg.data = (
            [float(n)] +
            [float(h) for h in self.h_bar] +
            [float(w) for w in self.w_bar] +
            [float(self.elev_map.coverage())]
        )
        self.terrain_pub.publish(msg)
        self.get_logger().info(
            f'Barrier: {n} steps | '
            f'h={[round(h) for h in self.h_bar]} | '
            f'w={[round(w) for w in self.w_bar]} | '
            f'coverage={self.elev_map.coverage():.1f}%'
        )


def main():
    rclpy.init()
    rclpy.spin(RealMissionPerception())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
