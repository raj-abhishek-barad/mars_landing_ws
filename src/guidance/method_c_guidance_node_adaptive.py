"""
Method C Guidance Node — Adaptive Barrier Version
===================================================
Same as before but now h_bar[], w_bar[] are updated in real time
from the terrain mapper node via /terrain/barrier_params topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from threading import Lock

# ============================================================
# GUIDANCE PARAMETERS (fixed)
# ============================================================
I_sp    = 225.0
g_e     = np.array([0.0, 0.0, -9.81])
g       = np.array([0.0, 0.0, -3.7114])
epsilon = 1e-6

l1 = np.array([1.0, 1.0, 1.0]) * 1.0
l2 = np.array([1.0, 1.0, 1.0]) * 4000.0
l3 = np.array([1.0, 1.0, 1.0]) * 380.0
lb = np.array([1.0, 1.0, 1.0]) * 0.4

rf = np.array([0.0, 0.0, 0.0])
vf = np.array([0.0, 0.0, 0.0])
m0 = 2000.0

alpha_ang = np.deg2rad(0.05)
radius    = 150.0

T_max   = 0.8 * 31000.0
T_min   = 0.2 * 31000.0
min_phi = -1.0
max_phi =  1.0
K1, phi1, k = 8.0, 10.0, 3.0

# ============================================================
# ADAPTIVE BARRIER STATE (shared, thread-safe)
# ============================================================
class BarrierState:
    """
    Holds current h_bar[], w_bar[] and derived quantities.
    Updated by terrain mapper, read by guidance loop.
    Thread-safe via Lock.
    """
    def __init__(self):
        self.lock   = Lock()
        # Safe initial guess — wide conservative barrier
        self._h_bar = [500.0, 1000.0, 1500.0]
        self._w_bar = [600.0,  800.0, 1400.0]
        self._n_steps    = 3
        self._coverage   = 0.0
        self._update_count = 0

    def update(self, h_bar, w_bar, coverage):
        with self.lock:
            self._h_bar    = list(h_bar)
            self._w_bar    = list(w_bar)
            self._n_steps  = len(h_bar)
            self._coverage = coverage
            self._update_count += 1

    def get(self):
        with self.lock:
            return (list(self._h_bar), list(self._w_bar),
                    self._n_steps, self._coverage, self._update_count)


# ============================================================
# GUIDANCE FUNCTIONS (now use adaptive barrier)
# ============================================================
def ZEM(r, v, t_go):
    return rf - (r + v * t_go + 0.5 * g * t_go**2)

def ZEV(v, t_go):
    return vf - (v + g * t_go)

def barriers_adaptive(x, y, z, h_bar, w_bar):
    """
    Same barrier logic but with variable-length h_bar[], w_bar[].
    Steps are sorted by w (innermost to outermost).
    """
    n = len(h_bar)
    assert len(w_bar) == n, "h_bar and w_bar must have same length"

    # Find which step zone we're in based on horizontal distance
    r_xy = np.sqrt(x**2 + y**2)

    # Determine rho1, rho2 (lateral barrier at this altitude z)
    # Walk through steps from innermost to outermost
    rho1 = w_bar[-1]  # default: outermost step
    for i in range(n):
        if z <= h_bar[i]:
            # We are below this step's height — use this step's width
            rho1 = w_bar[i]
            break
    rho2 = rho1

    # Determine rho3 (vertical barrier at this lateral position)
    rho3 = 0.0
    for i in range(n):
        if r_xy <= w_bar[i]:
            rho3 = h_bar[i]
            break
    else:
        # Beyond all steps — use last step height with slope
        rho3 = h_bar[-1] + np.tan(alpha_ang) * (r_xy - w_bar[-1])

    return rho1, rho2, rho3

def compute_penalty_terms_adaptive(r, v, h_bar, w_bar):
    rho1, rho2, z_barrier = barriers_adaptive(
        r[0], r[1], r[2], h_bar, w_bar)
    x_face = rho1 if r[0] >= 0 else -rho1
    y_face = rho2 if r[1] >= 0 else -rho2
    d = np.array([r[0]-x_face, r[1]-y_face, r[2]-z_barrier], dtype=float)

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

def method_c_command_adaptive(r, v, t_go, mass, r_ref, v_ref, h_bar, w_bar):
    t_go = max(t_go, 1e-6)
    zem  = ZEM(r, v, t_go)
    zev  = ZEV(v, t_go)
    d, p_dot = compute_penalty_terms_adaptive(r, v, h_bar, w_bar)

    a_nominal = (6.0/t_go**2)*zem - (2.0/t_go)*zev + (t_go**2/12.0)*p_dot

    e_r = r - r_ref
    e_v = v - v_ref
    S2  = e_v + (k/t_go)*e_r
    a_mss_raw = -K1 * np.tanh(S2/phi1)

    a_min = T_min / max(mass, epsilon)
    a_max = T_max / max(mass, epsilon)
    nom_norm = np.linalg.norm(a_nominal)
    mss_norm = np.linalg.norm(a_mss_raw)

    alpha_mss = max(0.0, min(1.0,
        (a_max-nom_norm)/(mss_norm+epsilon),
        (nom_norm-a_min)/(mss_norm+epsilon),
    )) if mss_norm > 1e-12 else 0.0

    a_cmd = a_nominal + alpha_mss * a_mss_raw
    a_norm = np.linalg.norm(a_cmd)
    if a_norm > 1e-12:
        if a_norm > a_max: a_cmd = (a_max/a_norm)*a_cmd
        elif a_norm < a_min: a_cmd = (a_min/a_norm)*a_cmd

    T = mass * a_cmd
    return T, a_cmd, S2

def estimate_tf(r, v):
    vz, z = v[2], r[2]
    if vz >= 0 or z <= 0:
        return 60.0
    t_free = (-vz + np.sqrt(max(vz**2 + 2*abs(g[2])*z, 0))) / abs(g[2])
    return np.clip(t_free * 1.5, 10.0, 300.0)


# ============================================================
# ROS2 NODE
# ============================================================
class MethodCAdaptiveNode(Node):
    def __init__(self):
        super().__init__('method_c_guidance_adaptive')

        self.barrier  = BarrierState()
        self.r        = None
        self.v        = None
        self.mass     = m0
        self.t        = 0.0
        self.r_ref    = None
        self.v_ref    = None
        self.tf       = None
        self.landed   = False
        self.initialized = False

        # Subscribers
        self.create_subscription(
            Odometry, '/model/lander/odometry',
            self.odom_cb, 10)

        # Adaptive barrier from terrain mapper
        self.create_subscription(
            Float64MultiArray, '/terrain/barrier_params',
            self.barrier_cb, 10)

        # Publishers
        self.thrust_pub = self.create_publisher(Wrench, '/lander/thrust', 10)
        self.state_pub  = self.create_publisher(
            Float64MultiArray, '/lander/state', 10)

        self.dt = 0.1
        self.create_timer(self.dt, self.guidance_loop)

        self.get_logger().info(
            'Method C Adaptive Guidance Node started.\n'
            'Waiting for odometry and terrain map...')

    def barrier_cb(self, msg):
        """
        Parse barrier params from terrain mapper:
        [n_steps, h0..hn, w0..wn, coverage]
        """
        data = list(msg.data)
        n    = int(data[0])
        h_bar    = data[1:1+n]
        w_bar    = data[1+n:1+2*n]
        coverage = data[1+2*n] if len(data) > 1+2*n else 0.0
        self.barrier.update(h_bar, w_bar, coverage)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        r_new = np.array([p.x, p.y, p.z])
        v_new = np.array([v.x, v.y, v.z])

        if not self.initialized:
            self.r     = r_new.copy()
            self.v     = v_new.copy()
            self.r_ref = r_new.copy()
            self.v_ref = v_new.copy()
            self.tf    = estimate_tf(self.r, self.v)
            self.initialized = True
            self.get_logger().info(
                f'Initialized:\n'
                f'  r0 = {np.round(self.r,2)}\n'
                f'  v0 = {np.round(self.v,2)}\n'
                f'  tf = {self.tf:.1f} s'
            )
        else:
            self.r = r_new.copy()
            self.v = v_new.copy()

    def guidance_loop(self):
        if self.landed or not self.initialized:
            return

        self.t += self.dt
        t_go    = max(self.tf - self.t, 1.0)

        # Get current barrier (adaptive, thread-safe)
        h_bar, w_bar, n_steps, coverage, n_updates = self.barrier.get()

        # Method C command with adaptive barrier
        T, a_cmd, S2 = method_c_command_adaptive(
            self.r, self.v, t_go, self.mass,
            self.r_ref, self.v_ref,
            h_bar, w_bar
        )

        # Fuel consumption
        mdot      = -np.linalg.norm(T) / (I_sp * np.linalg.norm(g_e))
        self.mass = max(self.mass + mdot * self.dt, 1.0)

        # Propagate reference
        a_ref      = a_cmd + g
        self.r_ref = self.r_ref + self.dt * self.v_ref
        self.v_ref = self.v_ref + self.dt * a_ref

        # Publish thrust
        wrench = Wrench()
        wrench.force.x = float(T[0])
        wrench.force.y = float(T[1])
        wrench.force.z = float(T[2])
        self.thrust_pub.publish(wrench)

        # Publish state
        state_msg = Float64MultiArray()
        state_msg.data = [
            self.r[0], self.r[1], self.r[2],
            self.v[0], self.v[1], self.v[2],
            float(self.mass), float(t_go),
            float(np.linalg.norm(T)),
        ]
        self.state_pub.publish(state_msg)

        # Console output
        pos_err = np.linalg.norm(self.r - rf)
        vel_err = np.linalg.norm(self.v - vf)
        self.get_logger().info(
            f't={self.t:6.1f}s | '
            f'r=[{self.r[0]:7.1f},{self.r[1]:7.1f},{self.r[2]:6.1f}]m | '
            f'v=[{self.v[0]:5.2f},{self.v[1]:5.2f},{self.v[2]:5.2f}]m/s | '
            f'|T|={np.linalg.norm(T):7.0f}N | '
            f'mass={self.mass:6.1f}kg | '
            f'steps={n_steps} | '
            f'map={coverage:.1f}% | '
            f'pos_err={pos_err:6.2f}m'
        )

        # Landing detection
        if self.r[2] <= 1.0 or (pos_err < 2.0 and vel_err < 1.0):
            self.get_logger().info(
                f'\n{"="*55}'
                f'\n  LANDED  at t = {self.t:.2f} s'
                f'\n  Final position : {np.round(self.r,3)}'
                f'\n  Final velocity : {np.round(self.v,3)}'
                f'\n  Fuel used      : {m0-self.mass:.2f} kg'
                f'\n  Pos error      : {pos_err:.3f} m'
                f'\n  Vel error      : {vel_err:.3f} m/s'
                f'\n  Map coverage   : {coverage:.1f}%'
                f'\n  Barrier steps  : {n_steps}'
                f'\n{"="*55}'
            )
            self.landed = True


def main():
    rclpy.init()
    node = MethodCAdaptiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
