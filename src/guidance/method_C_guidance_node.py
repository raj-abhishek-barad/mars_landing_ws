"""
Method C Guidance Node — Fixed + Improved
==========================================
Fixes from original:
  1. w.force.x/y/z = float(T) was wrong — T is a 3-vector
  2. ZEM/ZEV were missing rf, vf targets (assumed zero implicitly)
  3. terrain_cb stored w_bar but guidance loop never used it
  4. tf=120s hardcoded — now estimated from initial kinematics
  5. Thrust saturation was missing entirely
  6. Mass depletion (fuel burn) was missing

Improvements:
  - Full Method C: ZEM/ZEV + barrier penalty + MSS robust term
  - Adaptive barrier from perception node (/terrain/barriers)
  - Dynamic boundary layer phi scales with position uncertainty from EKF
  - Proper thrust saturation [T_min, T_max]
  - Fuel tracking (Isp-based mass flow)
  - Landing detection with final state printout
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from threading import Lock

# ============================================================
# CONSTANTS
# ============================================================
I_sp    = 225.0
g_e     = np.array([0.0, 0.0, -9.81])
g       = np.array([0.0, 0.0, -3.7114])
epsilon = 1e-6

rf = np.array([0.0, 0.0, 0.0])   # FIX: target position
vf = np.array([0.0, 0.0, 0.0])   # FIX: target velocity
m0 = 2000.0

# Barrier penalty weights
l1 = np.array([1.0, 1.0, 1.0]) * 1.0
l2 = np.array([1.0, 1.0, 1.0]) * 4000.0
l3 = np.array([1.0, 1.0, 1.0]) * 380.0
lb = np.array([1.0, 1.0, 1.0]) * 0.4
min_phi, max_phi = -1.0, 1.0

# Thrust limits
T_max = 0.8 * 31000.0   # 24800 N
T_min = 0.2 * 31000.0   # 6200  N

# Method C gains
K1   = 8.0
k    = 3.0


# ============================================================
# ADAPTIVE BARRIER STATE (thread-safe)
# ============================================================
class BarrierState:
    def __init__(self):
        self.lock     = Lock()
        self._h_bar   = [500.0, 1500.0]
        self._w_bar   = [600.0, 1400.0]
        self._n       = 2
        self._cov     = 0.0

    def update_from_terrain(self, data):
        """Parse [n, h0..hn, w0..wn, coverage] from perception node."""
        n = int(data[0])
        h = list(data[1:1+n])
        w = list(data[1+n:1+2*n])
        c = float(data[1+2*n]) if len(data) > 1+2*n else 0.0
        with self.lock:
            self._h_bar = h
            self._w_bar = w
            self._n     = n
            self._cov   = c

    def get(self):
        with self.lock:
            return list(self._h_bar), list(self._w_bar), self._n, self._cov


# ============================================================
# BARRIER FUNCTIONS (adaptive, variable steps)
# ============================================================
def barriers_adaptive(x, y, z, h_bar, w_bar):
    r_xy  = np.sqrt(x**2 + y**2)
    n     = len(h_bar)

    # Lateral barrier at altitude z
    rho1 = w_bar[-1]
    for i in range(n):
        if z <= h_bar[i]:
            rho1 = w_bar[i]
            break
    rho2 = rho1

    # Vertical barrier at lateral distance r_xy
    rho3 = h_bar[-1]
    for i in range(n):
        if r_xy <= w_bar[i]:
            rho3 = h_bar[i]
            break

    return rho1, rho2, rho3

def compute_penalty(r, v, h_bar, w_bar):
    rho1, rho2, z_barrier = barriers_adaptive(
        r[0], r[1], r[2], h_bar, w_bar)
    x_face = rho1 if r[0] >= 0 else -rho1
    y_face = rho2 if r[1] >= 0 else -rho2
    d      = np.array([r[0]-x_face, r[1]-y_face, r[2]-z_barrier])

    phi_x = np.clip((-l3[0]*d[0]*v[0]+l2[0]*v[0]**2)/(d[0]**2+l1[0]),
                    min_phi, max_phi)
    phi_y = np.clip((-l3[1]*d[1]*v[1]+l2[1]*v[1]**2)/(d[1]**2+l1[1]),
                    min_phi, max_phi)
    phi_z = np.clip((-l3[2]*d[2]*v[2]+l2[2]*v[2]**2)/(d[2]**2+l1[2]),
                    min_phi, max_phi)

    pd_x = (-lb[0]/2)*np.exp(lb[0]*phi_x)*(
        l3[0]*d[0]**2*v[0]-l1[0]*l2[0]*v[0]-l1[0]*l2[0]*v[0]**2
    )/(d[0]**2+l1[0])**2
    pd_y = (-lb[1]/2)*np.exp(lb[1]*phi_y)*(
        l3[1]*d[1]**2*v[1]-l1[1]*l2[1]*v[1]-l1[1]*l2[1]*v[1]**2
    )/(d[1]**2+l1[1])**2
    pd_z = (-lb[2]/2)*np.exp(lb[2]*phi_z)*(
        l3[2]*d[2]**2*v[2]-l1[2]*l2[2]*v[2]-l1[2]*l2[2]*v[2]**2
    )/(d[2]**2+l1[2])**2

    return d, np.array([pd_x, pd_y, pd_z])


# ============================================================
# GUIDANCE NODE
# ============================================================
class UncertaintyAwareGuidance(Node):
    def __init__(self):
        super().__init__('uncertainty_guidance')

        self.barrier = BarrierState()

        # State
        self.r, self.v     = None, None
        self.r_ref, self.v_ref = None, None
        self.P             = np.eye(6) * 0.1
        self.mass          = m0
        self.t             = 0.0
        self.tf            = None
        self.initialized   = False
        self.landed        = False

        # Dynamic boundary layer params
        self.phi_base = 10.0   # base boundary layer width
        self.beta     = 2.0    # uncertainty sensitivity

        # Subscribers
        self.create_subscription(
            Odometry, '/model/lander/odometry', self.odom_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/terrain/barriers', self.terrain_cb, 10)

        # Publisher
        self.thrust_pub = self.create_publisher(Wrench, '/lander/thrust', 10)

        self.dt = 0.1
        self.create_timer(self.dt, self.guidance_loop)
        self.get_logger().info(
            'Uncertainty-Aware Method C Guidance started. '
            'Waiting for odometry...')

    def terrain_cb(self, msg):
        """FIX: now actually parses and uses terrain data."""
        if len(msg.data) < 3:
            return
        self.barrier.update_from_terrain(msg.data)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        r_new = np.array([p.x, p.y, p.z])
        v_new = np.array([v.x, v.y, v.z])

        # FIX: extract covariance properly
        self.P = np.array(msg.pose.covariance).reshape(6, 6)

        if not self.initialized:
            self.r     = r_new.copy()
            self.v     = v_new.copy()
            self.r_ref = r_new.copy()
            self.v_ref = v_new.copy()

            # FIX: estimate tf from kinematics instead of hardcoding 120s
            vz, z = v_new[2], r_new[2]
            if vz < 0 and z > 0:
                t_free = (-vz + np.sqrt(
                    max(vz**2 + 2*abs(g[2])*z, 0))) / abs(g[2])
                self.tf = float(np.clip(t_free * 1.5, 10.0, 300.0))
            else:
                self.tf = 120.0

            self.initialized = True
            self.get_logger().info(
                f'Initialized:\n'
                f'  r0 = {np.round(r_new,1)}\n'
                f'  v0 = {np.round(v_new,2)}\n'
                f'  tf = {self.tf:.1f} s'
            )
        else:
            self.r = r_new.copy()
            self.v = v_new.copy()

    def guidance_loop(self):
        if self.landed or not self.initialized:
            return

        self.t  += self.dt
        t_go     = max(self.tf - self.t, 1.0)

        # Get current barrier
        h_bar, w_bar, n_steps, coverage = self.barrier.get()

        # 1. ZEM/ZEV nominal guidance (FIX: include rf, vf)
        zem = rf - (self.r + self.v * t_go + 0.5 * g * t_go**2)
        zev = vf - (self.v + g * t_go)
        a_nom = (6.0/t_go**2)*zem - (2.0/t_go)*zev

        # 2. Barrier penalty term (FIX: was completely missing)
        d, p_dot = compute_penalty(self.r, self.v, h_bar, w_bar)
        a_barrier = (t_go**2 / 12.0) * p_dot

        # 3. MSS robust term with dynamic boundary layer
        e_r = self.r - self.r_ref
        e_v = self.v - self.v_ref
        S   = e_v + (k / t_go) * e_r

        # Dynamic phi: widens when position uncertainty is high
        pos_uncertainty = np.trace(self.P[:3, :3])
        dynamic_phi     = self.phi_base + self.beta * pos_uncertainty
        a_mss           = -K1 * np.tanh(S / dynamic_phi)

        # 4. Combined command
        a_cmd  = a_nom + a_barrier + a_mss

        # 5. FIX: thrust saturation
        a_min  = T_min / max(self.mass, epsilon)
        a_max  = T_max / max(self.mass, epsilon)
        a_norm = np.linalg.norm(a_cmd)
        if a_norm > 1e-12:
            if a_norm > a_max:
                a_cmd = (a_max / a_norm) * a_cmd
            elif a_norm < a_min:
                a_cmd = (a_min / a_norm) * a_cmd

        T = self.mass * a_cmd

        # 6. FIX: fuel tracking (was missing entirely)
        mdot       = -np.linalg.norm(T) / (I_sp * np.linalg.norm(g_e))
        self.mass  = max(self.mass + mdot * self.dt, 1.0)

        # 7. Propagate reference
        self.r_ref = self.r_ref + self.dt * self.v_ref
        self.v_ref = self.v_ref + self.dt * (a_cmd + g)

        # 8. FIX: publish thrust correctly (T is a 3-vector)
        w = Wrench()
        w.force.x = float(T[0])
        w.force.y = float(T[1])
        w.force.z = float(T[2])
        self.thrust_pub.publish(w)

        # Console output
        pos_err = np.linalg.norm(self.r - rf)
        vel_err = np.linalg.norm(self.v - vf)
        self.get_logger().info(
            f't={self.t:6.1f}s | '
            f'r=[{self.r[0]:7.1f},{self.r[1]:7.1f},{self.r[2]:6.1f}]m | '
            f'v=[{self.v[0]:5.2f},{self.v[1]:5.2f},{self.v[2]:5.2f}]m/s | '
            f'|T|={np.linalg.norm(T):7.0f}N | '
            f'mass={self.mass:6.1f}kg | '
            f'phi={dynamic_phi:.2f} | '
            f'steps={n_steps} | '
            f'map={coverage:.1f}% | '
            f'pos_err={pos_err:.2f}m'
        )

        # Landing detection
        if self.r[2] <= 1.0 or (pos_err < 2.0 and vel_err < 1.0):
            self.get_logger().info(
                f'\n{"="*55}'
                f'\n  LANDED at t = {self.t:.2f} s'
                f'\n  Position : {np.round(self.r, 3)}'
                f'\n  Velocity : {np.round(self.v, 3)}'
                f'\n  Fuel used: {m0 - self.mass:.2f} kg'
                f'\n  Pos error: {pos_err:.3f} m'
                f'\n  Vel error: {vel_err:.3f} m/s'
                f'\n  Map coverage: {coverage:.1f}%'
                f'\n{"="*55}'
            )
            self.landed = True


def main():
    rclpy.init()
    rclpy.spin(UncertaintyAwareGuidance())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
