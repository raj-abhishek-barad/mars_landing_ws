"""
Terrain Mapper — Camera Version
=================================
Supports two modes:
  MODE A: Depth camera  → direct depth image → point cloud → elevation map
  MODE B: Monocular     → RGB image → MiDaS depth estimation → elevation map

Both feed into the same ElevationMap + AdaptiveStepFitter pipeline.

Architecture:
  Camera (5Hz)
      ↓
  DepthEstimator (MODE A: passthrough | MODE B: MiDaS neural net)
      ↓ depth image (metric, meters)
  DepthToPointCloud (using camera intrinsics + lander pose)
      ↓ 3D points in world frame
  ElevationMap (cumulative radial bins)
      ↓
  AdaptiveStepFitter (variable steps, safety margin)
      ↓
  /terrain/barrier_params → guidance node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
from threading import Lock
import cv2

# Try importing MiDaS for monocular mode
try:
    import torch
    import torchvision.transforms as transforms
    MIDAS_AVAILABLE = True
except ImportError:
    MIDAS_AVAILABLE = False


# ============================================================
# CAMERA INTRINSICS
# ============================================================
class CameraIntrinsics:
    """
    Pinhole camera model.
    For Gazebo camera with FOV=1.2rad, resolution 320x240.
    """
    def __init__(self, width=320, height=240, hfov=1.2):
        self.width  = width
        self.height = height
        self.hfov   = hfov
        self.fx     = (width / 2.0) / np.tan(hfov / 2.0)
        self.fy     = self.fx  # square pixels
        self.cx     = width  / 2.0
        self.cy     = height / 2.0

    def deproject(self, depth_image):
        """
        Convert depth image to 3D point cloud in camera frame.
        depth_image: HxW float array, depth in meters.
        Returns: Nx3 array of (x,y,z) in camera frame.
        """
        H, W = depth_image.shape
        u = np.arange(W)
        v = np.arange(H)
        uu, vv = np.meshgrid(u, v)

        z = depth_image.flatten()
        x = (uu.flatten() - self.cx) * z / self.fx
        y = (vv.flatten() - self.cy) * z / self.fy

        pts = np.stack([x, y, z], axis=1)
        # Filter invalid depths
        valid = np.isfinite(z) & (z > 0.1) & (z < 2999.0)
        return pts[valid]


# ============================================================
# MIDAS DEPTH ESTIMATOR (monocular mode)
# ============================================================
class MiDaSEstimator:
    """
    Estimates metric depth from a single RGB image using MiDaS.
    Uses scale/shift alignment with known altitude for metric recovery.
    """
    def __init__(self, model_type="MiDaS_small"):
        if not MIDAS_AVAILABLE:
            raise RuntimeError("torch/torchvision not installed")

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.hub.load(
            "intel-isl/MiDaS", model_type, trust_repo=True)
        self.model.to(self.device)
        self.model.eval()

        midas_transforms = torch.hub.load(
            "intel-isl/MiDaS", "transforms", trust_repo=True)
        if model_type == "MiDaS_small":
            self.transform = midas_transforms.small_transform
        else:
            self.transform = midas_transforms.dpt_transform

        print(f"[MiDaS] Loaded {model_type} on {self.device}")

    def estimate(self, rgb_image, altitude_hint=None):
        """
        rgb_image: HxWx3 uint8 numpy array (BGR from cv2)
        altitude_hint: known lander altitude (m) for scale recovery
        Returns: HxW float depth image in meters
        """
        img_rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img_rgb).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=rgb_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_rel = prediction.cpu().numpy()  # relative, not metric

        # Scale recovery using altitude hint
        # The nadir (center) pixel should be at ~altitude distance
        if altitude_hint is not None and altitude_hint > 0:
            h, w = depth_rel.shape
            center_val = depth_rel[h//2, w//2]
            if center_val > 1e-6:
                scale = altitude_hint / center_val
                depth_metric = depth_rel * scale
            else:
                depth_metric = depth_rel
        else:
            depth_metric = depth_rel

        return depth_metric.astype(np.float32)


# ============================================================
# ELEVATION MAP (same as lidar version)
# ============================================================
class ElevationMap:
    def __init__(self, r_max=3000.0, bin_size=20.0):
        self.r_max    = r_max
        self.bin_size = bin_size
        self.n_bins   = int(r_max / bin_size) + 1
        self.elevation = np.full(self.n_bins, -np.inf)
        self.observed  = np.zeros(self.n_bins, dtype=bool)
        self.lock      = Lock()
        self.total_points_added = 0

    def update(self, points_world):
        with self.lock:
            for pt in points_world:
                x, y, z = pt
                r = np.sqrt(x**2 + y**2)
                if r > self.r_max or r < 1.0:
                    continue
                idx = min(int(r / self.bin_size), self.n_bins - 1)
                if z > self.elevation[idx]:
                    self.elevation[idx] = z
                    self.observed[idx]  = True
            self.total_points_added += len(points_world)

    def get_observed(self):
        with self.lock:
            mask  = self.observed.copy()
            radii = np.arange(self.n_bins)[mask] * self.bin_size
            elev  = self.elevation[mask].copy()
        return radii, elev

    def coverage_percent(self):
        return 100.0 * self.observed.sum() / self.n_bins


# ============================================================
# ADAPTIVE STEP FITTER (same as lidar version)
# ============================================================
class AdaptiveStepFitter:
    def __init__(self, merge_tol=30.0, safety_margin=50.0,
                 n_steps_min=2, n_steps_max=20, w_min=50.0):
        self.merge_tol     = merge_tol
        self.safety_margin = safety_margin
        self.n_steps_min   = n_steps_min
        self.n_steps_max   = n_steps_max
        self.w_min         = w_min

    def fit(self, radii, elevations):
        if len(radii) < 2:
            return [500.0, 1000.0], [600.0, 800.0]

        order    = np.argsort(radii)
        r_sorted = radii[order]
        z_safe   = elevations[order] + self.safety_margin

        # Greedy merge
        groups = []
        g_r, g_z = [r_sorted[0]], [z_safe[0]]
        for i in range(1, len(r_sorted)):
            if abs(z_safe[i] - np.mean(g_z)) < self.merge_tol:
                g_r.append(r_sorted[i])
                g_z.append(z_safe[i])
            else:
                groups.append((max(g_r), max(g_z)))
                g_r, g_z = [r_sorted[i]], [z_safe[i]]
        groups.append((max(g_r), max(g_z)))

        # Cap steps
        if len(groups) > self.n_steps_max:
            idx    = np.round(np.linspace(
                0, len(groups)-1, self.n_steps_max)).astype(int)
            groups = [groups[i] for i in idx]

        while len(groups) < self.n_steps_min:
            w_last, h_last = groups[-1]
            groups.append((w_last * 1.5, h_last))

        w_bar, h_bar, w_prev = [], [], self.w_min
        for w, h in groups:
            w = max(w, w_prev + self.w_min)
            w_bar.append(float(w))
            h_bar.append(float(max(h, 0.0)))
            w_prev = w

        return h_bar, w_bar


# ============================================================
# COORDINATE TRANSFORM
# ============================================================
def transform_to_world(pts_cam, lander_pos, lander_quat,
                       cam_pose_offset=None):
    """
    pts_cam: Nx3 in camera frame
    cam_pose_offset: 4x4 transform from camera to body frame
                     (accounts for camera mounting position/orientation)
    """
    qx, qy, qz, qw = lander_quat
    R_body_world = np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx**2+qy**2)],
    ])

    # Camera is mounted pointing down (rotated 90° around X in SDF)
    # In camera frame: Z = forward (down), X = right, Y = down
    # In body frame:   Z = up, so camera Z maps to body -Z
    R_cam_body = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0],
    ])

    pts_body  = (R_cam_body @ pts_cam.T).T
    pts_world = (R_body_world @ pts_body.T).T + lander_pos
    return pts_world


# ============================================================
# ROS2 NODE
# ============================================================
class TerrainMapperCameraNode(Node):
    def __init__(self):
        super().__init__('terrain_mapper_camera')

        # Declare mode parameter: 'depth' or 'mono'
        self.declare_parameter('mode', 'mono')
        self.mode = self.get_parameter('mode').value
        self.get_logger().info(f'Terrain mapper mode: {self.mode}')

        self.bridge   = CvBridge()
        self.elev_map = ElevationMap(r_max=3000.0, bin_size=20.0)
        self.fitter   = AdaptiveStepFitter(
            merge_tol=30.0, safety_margin=50.0,
            n_steps_min=2,  n_steps_max=20)

        self.lander_pos  = np.array([0.0, 0.0, 200.0])
        self.lander_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.h_bar = [500.0, 1000.0, 1500.0]
        self.w_bar = [600.0,  800.0, 1400.0]
        self.cam_intrinsics = CameraIntrinsics(
            width=320, height=240, hfov=1.2)

        # MiDaS for monocular mode
        self.midas = None
        if self.mode == 'mono':
            if MIDAS_AVAILABLE:
                self.get_logger().info('Loading MiDaS model...')
                try:
                    self.midas = MiDaSEstimator(model_type="MiDaS_small")
                    self.get_logger().info('MiDaS loaded successfully.')
                except Exception as e:
                    self.get_logger().warn(
                        f'MiDaS failed to load: {e}\n'
                        f'Falling back to altitude-based flat depth estimate.')
            else:
                self.get_logger().warn(
                    'torch not installed. '
                    'Using flat depth estimate (altitude only).')

        # Subscribers
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry, '/model/lander/odometry', self.odom_cb, 10)

        if self.mode == 'depth':
            self.create_subscription(
                Image, '/lander/depth/image',
                self.depth_image_cb, 10)
        else:
            self.create_subscription(
                Image, '/lander/camera/image_raw',
                self.mono_image_cb, 10)

        # Publisher
        self.barrier_pub = self.create_publisher(
            Float64MultiArray, '/terrain/barrier_params', 10)

        self.create_timer(0.2, self.publish_barrier)
        self.get_logger().info('Terrain Mapper (Camera) ready.')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.lander_pos  = np.array([p.x, p.y, p.z])
        self.lander_quat = np.array([q.x, q.y, q.z, q.w])

    def depth_image_cb(self, msg):
        """MODE A: Direct depth image from depth camera."""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            depth = np.array(depth, dtype=np.float32)
            self._process_depth(depth)
        except Exception as e:
            self.get_logger().warn(f'depth_image_cb: {e}')

    def mono_image_cb(self, msg):
        """MODE B: RGB image → MiDaS depth → process."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            altitude = self.lander_pos[2]

            if self.midas is not None:
                # Full MiDaS depth estimation
                depth = self.midas.estimate(rgb, altitude_hint=altitude)
            else:
                # Fallback: assume flat terrain at current altitude
                # This gives a flat elevation map — not ideal but safe
                depth = np.full(
                    (rgb.shape[0], rgb.shape[1]),
                    altitude, dtype=np.float32)

            self._process_depth(depth)
        except Exception as e:
            self.get_logger().warn(f'mono_image_cb: {e}')

    def _process_depth(self, depth_image):
        """Common processing: depth → 3D points → elevation map → barrier."""
        # Subsample for speed (every 4th pixel)
        depth_sub = depth_image[::4, ::4]

        # Deproject to camera-frame point cloud
        pts_cam = self.cam_intrinsics.deproject(depth_sub)
        if len(pts_cam) == 0:
            return

        # Transform to world frame
        pts_world = transform_to_world(
            pts_cam, self.lander_pos, self.lander_quat)

        # Update elevation map
        self.elev_map.update(pts_world)

        # Refit barrier
        radii, elevs = self.elev_map.get_observed()
        if len(radii) >= 2:
            self.h_bar, self.w_bar = self.fitter.fit(radii, elevs)

        self.get_logger().info(
            f'Camera processed: {len(pts_world)} pts | '
            f'map={self.elev_map.coverage_percent():.1f}% | '
            f'steps={len(self.h_bar)} | '
            f'alt={self.lander_pos[2]:.0f}m'
        )

    def publish_barrier(self):
        n   = len(self.h_bar)
        msg = Float64MultiArray()
        msg.data = (
            [float(n)] +
            [float(h) for h in self.h_bar] +
            [float(w) for w in self.w_bar] +
            [float(self.elev_map.coverage_percent())]
        )
        self.barrier_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TerrainMapperCameraNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
