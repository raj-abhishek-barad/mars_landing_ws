"""
Method C Guidance Node — Adaptive Barrier + Attitude Control
=============================================================
Adds a two-loop control structure on top of the original:

  OUTER LOOP (10 Hz): Method C translational guidance
      Computes desired thrust vector T_des in WORLD frame
      from ZEM/ZEV + barrier penalty + MSS augmentation.

  INNER LOOP (100 Hz): Attitude PD controller
      Aligns body +Z axis with T_des direction.
      Computes corrective torque τ from quaternion error + ω damping.
      Publishes full Wrench (force + torque) at IMU rate.

Key design decisions
---------------------
  1. Thrust magnitude is set by guidance; attitude controller only
     rotates the vehicle — it does not change |T|.
  2. Force is applied in WORLD frame (consistent with ApplyLinkWrench
     default behaviour in Gazebo).
  3. Torque is applied in BODY frame (that is what makes physical sense
     for a thruster/RCS system rotating the vehicle).
  4. Angular velocity comes from IMU (/imu/data) at 100 Hz, which gives
     the inner loop its full bandwidth.
  5. Quaternion comes from odometry (/model/lander/odometry) — the same
     source used for position/velocity — to keep frames consistent.

Attitude controller maths
--------------------------
  Given:
    q_curr  — current orientation quaternion [qx, qy, qz, qw]
    T_des   — desired thrust direction (world frame)
    ω       — body angular velocity (body frame, from IMU)

  Step 1 — Desired body axis:
    ẑ_des  = T_des / |T_des|          (desired body +Z in world frame)

  Step 2 — Current body +Z in world frame:
    ẑ_curr = R(q_curr) · [0, 0, 1]ᵀ

  Step 3 — Rotation error (axis-angle):
    e_axis = ẑ_curr × ẑ_des           (rotation axis, world frame)
    e_angle = arccos(ẑ_curr · ẑ_des)  (rotation magnitude)

  Step 4 — Express error axis in body frame:
    e_body = R(q_curr)ᵀ · e_axis

  Step 5 — PD torque in body frame:
    τ = Kp · e_angle · e_body − Kd · ω

Tuning guidance
---------------
  Kp : proportional gain [N·m/rad]
       Start at I_min/5 where I_min = min(Ixx, Iyy, Izz).
       For I=1333 kg·m², start at ~270 N·m/rad.
  Kd : derivative gain [N·m·s/rad]
       Rule of thumb: Kd ≈ 2*sqrt(Kp * I_min) for critical damping.
       For Kp=270, I=1333: Kd ≈ 2*sqrt(270*1333) ≈ 1197 → round to 1200.
  τ_max : torque saturation [N·m]
       Keep below what RCS thrusters can physically produce.
       Set to 5000 N·m as a reasonable upper bound for a 2000 kg lander.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
from threading import Lock

# ============================================================
# GUIDANCE PARAMETERS
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
# ATTITUDE CONTROLLER PARAMETERS
# ============================================================
# Inertia of the lander body (kg·m²) — must match model.sdf
# For solid box 2x2x2m at 2000kg: I = (1/6)*m*a² = 1333.3 kg·m²
I_BODY = np.diag([1333.3, 1333.3, 1333.3])

ATT_KP    = 270.0    # N·m/rad — proportional gain
ATT_KD    = 1200.0   # N·m·s   — derivative (damping) gain
ATT_TMAX  = 5000.0   # N·m     — torque saturation per axis

# Pointing tolerance: if misalignment < this, suppress lateral thrust
# to avoid thruster firing sideways during large attitude errors
ATT_ALIGN_TOL = np.deg2rad(15.0)  # 15 degrees


# ============================================================
# HELPER: quaternion → rotation matrix
# ============================================================
def quat_to_R(q):
    """
    q = [qx, qy, qz, qw]
    Returns 3x3 rotation matrix R such that v_world = R @ v_body
    """
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


# ============================================================
# ATTITUDE CONTROLLER
# ============================================================
def attitude_control(q_curr, T_des, omega_body):
    """
    Quaternion PD attitude controller.

    Parameters
    ----------
    q_curr     : array [qx, qy, qz, qw] — current orientation
    T_des      : array [Tx, Ty, Tz]     — desired thrust vector (world frame)
    omega_body : array [wx, wy, wz]     — current body angular velocity
                                          (body frame, from IMU)

    Returns
    -------
    tau        : array [tx, ty, tz]     — torque command (body frame)
    angle_err  : float                  — pointing error in radians
    aligned    : bool                   — True if within ATT_ALIGN_TOL
    """
    T_norm = np.linalg.norm(T_des)
    if T_norm < 1.0:
        # No meaningful thrust direction — just damp rotation
        tau = -ATT_KD * omega_body
        tau = np.clip(tau, -ATT_TMAX, ATT_TMAX)
        return tau, np.pi, False

    # Desired body +Z direction in world frame
    z_des_world = T_des / T_norm

    # Current body +Z in world frame
    R = quat_to_R(q_curr)
    z_curr_world = R[:, 2]   # third column = body Z in world

    # Rotation error
    cos_err = np.clip(np.dot(z_curr_world, z_des_world), -1.0, 1.0)
    angle_err = np.arccos(cos_err)

    e_cross_world = np.cross(z_curr_world, z_des_world)
    cross_norm = np.linalg.norm(e_cross_world)

    if cross_norm > 1e-6:
        e_axis_world = e_cross_world / cross_norm
    else:
        # Already aligned or 180° flip — use arbitrary perpendicular axis
        e_axis_world = np.array([1.0, 0.0, 0.0]) if abs(z_curr_world[0]) < 0.9 \
                       else np.array([0.0, 1.0, 0.0])

    # Express error axis in body frame: e_body = Rᵀ · e_world
    e_axis_body = R.T @ e_axis_world

    # PD torque in body frame
    tau = ATT_KP * angle_err * e_axis_body - ATT_KD * omega_body
    tau = np.clip(tau, -ATT_TMAX, ATT_TMAX)

    aligned = angle_err < ATT_ALIGN_TOL
    return tau, angle_err, aligned


# ============================================================
# ADAPTIVE BARRIER STATE (thread-safe)
# ============================================================
class BarrierState:
    def __init__(self):
        self.lock          = Lock()
        self._h_bar        = [500.0, 1000.0, 1500.0]
        self._w_bar        = [600.0,  800.0, 1400.0]
        self._n_steps      = 3
        self._coverage     = 0.0
        self._update_count = 0

    def update(self, h_bar, w_bar, coverage):
        with self.lock:
            self._h_bar        = list(h_bar)
            self._w_bar        = list(w_bar)
            self._n_steps      = len(h_bar)
            self._coverage     = coverage
            self._update_count += 1

    def get(self):
        with self.lock:
            return (list(self._h_bar), list(self._w_bar),
                    self._n_steps, self._coverage, self._update_count)


# ============================================================
# ATTITUDE STATE (thread-safe, updated at IMU rate 100 Hz)
# ============================================================
class AttitudeState:
    def __init__(self):
        self.lock  = Lock()
        self._q    = np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion
        self._omega = np.zeros(3)

    def update_omega(self, omega):
        with self.lock:
            self._omega = np.array(omega, dtype=float)

    def update_q(self, q):
        with self.lock:
            q = np.array(q, dtype=float)
            n = np.linalg.norm(q)
            self._q = q / n if n > 1e-9 else np.array([0., 0., 0., 1.])

    def get(self):
        with self.lock:
            return self._q.copy(), self._omega.copy()


# ============================================================
# GUIDANCE FUNCTIONS
# ============================================================
def ZEM(r, v, t_go):
    return rf - (r + v * t_go + 0.5 * g * t_go**2)

def ZEV(v, t_go):
    return vf - (v + g * t_go)

def barriers_adaptive(x, y, z, h_bar, w_bar):
    n    = len(h_bar)
    r_xy = np.sqrt(x**2 + y**2)

    rho1 = w_bar[-1]
    for i in range(n):
        if z <= h_bar[i]:
            rho1 = w_bar[i]
            break
    rho2 = rho1

    rho3 = 0.0
    for i in range(n):
        if r_xy <= w_bar[i]:
            rho3 = h_bar[i]
            break
    else:
        rho3 = h_bar[-1] + np.tan(alpha_ang) * (r_xy - w_bar[-1])

    return rho1, rho2, rho3

def compute_penalty_terms_adaptive(r, v, h_bar, w_bar):
    rho1, rho2, z_barrier = barriers_adaptive(r[0], r[1], r[2], h_bar, w_bar)
    x_face = rho1 if r[0] >= 0 else -rho1
    y_face = rho2 if r[1] >= 0 else -rho2
    d = np.array([r[0]-x_face, r[1]-y_face, r[2]-z_barrier], dtype=float)

    phi_x = np.clip((-l3[0]*d[0]*v[0] + l2[0]*v[0]**2) / (d[0]**2 + l1[0]),
                    min_phi, max_phi)
    phi_y = np.clip((-l3[1]*d[1]*v[1] + l2[1]*v[1]**2) / (d[1]**2 + l1[1]),
                    min_phi, max_phi)
    phi_z = np.clip((-l3[2]*d[2]*v[2] + l2[2]*v[2]**2) / (d[2]**2 + l1[2]),
                    min_phi, max_phi)

    pd_x = (-lb[0]/2)*np.exp(lb[0]*phi_x)*(
        l3[0]*d[0]**2*v[0] - l1[0]*l2[0]*v[0] - l1[0]*l2[0]*v[0]**2
    ) / (d[0]**2 + l1[0])**2
    pd_y = (-lb[1]/2)*np.exp(lb[1]*phi_y)*(
        l3[1]*d[1]**2*v[1] - l1[1]*l2[1]*v[1] - l1[1]*l2[1]*v[1]**2
    ) / (d[1]**2 + l1[1])**2
    pd_z = (-lb[2]/2)*np.exp(lb[2]*phi_z)*(
        l3[2]*d[2]**2*v[2] - l1[2]*l2[2]*v[2] - l1[2]*l2[2]*v[2]**2
    ) / (d[2]**2 + l1[2])**2

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
        (a_max - nom_norm) / (mss_norm + epsilon),
        (nom_norm - a_min) / (mss_norm + epsilon),
    )) if mss_norm > 1e-12 else 0.0

    a_cmd = a_nominal + alpha_mss * a_mss_raw
    a_norm = np.linalg.norm(a_cmd)
    if a_norm > 1e-12:
        if a_norm > a_max:   a_cmd = (a_max / a_norm) * a_cmd
        elif a_norm < a_min: a_cmd = (a_min / a_norm) * a_cmd

    T = mass * a_cmd
    return T, a_cmd, S2

def estimate_tf(r, v):
    vz, z = v[2], r[2]
    if vz >= 0 or z <= 0:
        return 60.0
    t_free = (-vz + np.sqrt(max(vz**2 + 2*abs(g[2])*z, 0.0))) / abs(g[2])
    return np.clip(t_free * 1.5, 10.0, 300.0)


# ============================================================
# ROS2 NODE
# ============================================================
class MethodCAdaptiveNode(Node):
    def __init__(self):
        super().__init__('method_c_guidance_adaptive')

        self.barrier  = BarrierState()
        self.att      = AttitudeState()

        # Translational state
        self.r           = None
        self.v           = None
        self.mass        = m0
        self.t           = 0.0
        self.r_ref       = None
        self.v_ref       = None
        self.tf          = None
        self.landed      = False
        self.initialized = False

        # Last desired thrust direction — shared between guidance (10 Hz)
        # and attitude controller (100 Hz)
        self._T_des      = np.array([0.0, 0.0, T_min])  # safe default: hover
        self._T_des_lock = Lock()

        # --------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------
        self.create_subscription(
            Odometry, '/model/lander/odometry',
            self.odom_cb, 10)

        self.create_subscription(
            Float64MultiArray, '/terrain/barrier_params',
            self.barrier_cb, 10)

        # IMU at 100 Hz — drives the attitude inner loop
        self.create_subscription(
            Imu, '/imu/data',
            self.imu_cb, 10)

        # --------------------------------------------------------
        # Publishers
        # --------------------------------------------------------
        self.thrust_pub = self.create_publisher(
            Wrench, '/lander/thrust', 10)
        self.state_pub  = self.create_publisher(
            Float64MultiArray, '/lander/state', 10)
        self.att_pub    = self.create_publisher(
            Float64MultiArray, '/lander/attitude_error', 10)

        # --------------------------------------------------------
        # Timers
        # --------------------------------------------------------
        # Guidance outer loop: 10 Hz
        self.dt_guidance = 0.1
        self.create_timer(self.dt_guidance, self.guidance_loop)

        self.get_logger().info(
            'Method C + Attitude Control node started.\n'
            f'  ATT_KP={ATT_KP} N·m/rad | ATT_KD={ATT_KD} N·m·s | '
            f'ATT_TMAX={ATT_TMAX} N·m\n'
            'Waiting for odometry, IMU and terrain map...')

    # ----------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------
    def barrier_cb(self, msg):
        data = list(msg.data)
        n    = int(data[0])
        h_bar    = data[1:1+n]
        w_bar    = data[1+n:1+2*n]
        coverage = data[1+2*n] if len(data) > 1+2*n else 0.0
        self.barrier.update(h_bar, w_bar, coverage)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation

        r_new = np.array([p.x, p.y, p.z])
        v_new = np.array([v.x, v.y, v.z])

        # Update quaternion from odometry (lower rate but same source as r, v)
        self.att.update_q([q.x, q.y, q.z, q.w])

        if not self.initialized:
            self.r     = r_new.copy()
            self.v     = v_new.copy()
            self.r_ref = r_new.copy()
            self.v_ref = v_new.copy()
            self.tf    = estimate_tf(self.r, self.v)
            self.initialized = True
            self.get_logger().info(
                f'Initialized:\n'
                f'  r0 = {np.round(self.r, 2)}\n'
                f'  v0 = {np.round(self.v, 2)}\n'
                f'  tf = {self.tf:.1f} s')
        else:
            self.r = r_new.copy()
            self.v = v_new.copy()

    def imu_cb(self, msg):
        """
        Inner loop — runs at IMU rate (100 Hz).

        1. Update angular velocity from IMU.
        2. Read latest desired thrust direction from guidance.
        3. Compute attitude PD torque.
        4. Publish Wrench (force from guidance + torque from attitude ctrl).

        This runs independently of the guidance timer so that angular
        velocity damping continues even between guidance updates.
        """
        if not self.initialized:
            return

        # Update omega from IMU (body frame)
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.att.update_omega(omega)

        # Get current attitude
        q_curr, omega_body = self.att.get()

        # Get latest desired thrust direction
        with self._T_des_lock:
            T_des = self._T_des.copy()

        # Attitude controller
        tau, angle_err, aligned = attitude_control(q_curr, T_des, omega_body)

        # If attitude error is large, scale down lateral thrust components
        # to avoid the engine firing sideways while rotating.
        # Vertical (Z) thrust is kept to maintain altitude.
        T_applied = T_des.copy()
        if not aligned:
            # Blend: full thrust only when aligned, only Z component otherwise
            blend      = np.cos(angle_err / 2.0)**2  # 1 when aligned, 0 at 90°
            T_vertical = np.array([0.0, 0.0, T_des[2]])
            T_applied  = blend * T_des + (1.0 - blend) * T_vertical

        # Publish Wrench
        # Force : world frame  (ApplyLinkWrench default)
        # Torque: body frame   (rotates the vehicle)
        wrench = Wrench()
        wrench.force.x  = float(T_applied[0])
        wrench.force.y  = float(T_applied[1])
        wrench.force.z  = float(T_applied[2])
        wrench.torque.x = float(tau[0])
        wrench.torque.y = float(tau[1])
        wrench.torque.z = float(tau[2])
        self.thrust_pub.publish(wrench)

        # Publish attitude diagnostics
        att_msg = Float64MultiArray()
        att_msg.data = [
            float(np.degrees(angle_err)),   # pointing error [deg]
            float(tau[0]),
            float(tau[1]),
            float(tau[2]),                  # torque components [N·m]
            float(np.linalg.norm(omega)),   # angular speed [rad/s]
            float(1.0 if aligned else 0.0), # alignment flag
        ]
        self.att_pub.publish(att_msg)

    # ----------------------------------------------------------
    # GUIDANCE OUTER LOOP (10 Hz)
    # ----------------------------------------------------------
    def guidance_loop(self):
        if self.landed or not self.initialized:
            return

        self.t  += self.dt_guidance
        t_go     = max(self.tf - self.t, 1.0)

        h_bar, w_bar, n_steps, coverage, n_updates = self.barrier.get()

        T, a_cmd, S2 = method_c_command_adaptive(
            self.r, self.v, t_go, self.mass,
            self.r_ref, self.v_ref,
            h_bar, w_bar
        )

        # Share desired thrust with inner loop (thread-safe)
        with self._T_des_lock:
            self._T_des = T.copy()

        # Fuel consumption
        mdot      = -np.linalg.norm(T) / (I_sp * np.linalg.norm(g_e))
        self.mass = max(self.mass + mdot * self.dt_guidance, 1.0)

        # Propagate reference trajectory
        a_ref      = a_cmd + g
        self.r_ref = self.r_ref + self.dt_guidance * self.v_ref
        self.v_ref = self.v_ref + self.dt_guidance * a_ref

        # Diagnostics
        q_curr, omega = self.att.get()
        _, angle_err, aligned = attitude_control(q_curr, T, omega)

        pos_err = np.linalg.norm(self.r - rf)
        vel_err = np.linalg.norm(self.v - vf)

        self.get_logger().info(
            f't={self.t:6.1f}s | '
            f'r=[{self.r[0]:7.1f},{self.r[1]:7.1f},{self.r[2]:6.1f}]m | '
            f'v=[{self.v[0]:5.2f},{self.v[1]:5.2f},{self.v[2]:5.2f}]m/s | '
            f'|T|={np.linalg.norm(T):7.0f}N | '
            f'mass={self.mass:6.1f}kg | '
            f'att_err={np.degrees(angle_err):5.1f}deg | '
            f'{"ALIGNED" if aligned else "ROTATING":>8s} | '
            f'steps={n_steps} | map={coverage:.1f}% | '
            f'pos_err={pos_err:6.2f}m'
        )

        # Publish state
        state_msg = Float64MultiArray()
        state_msg.data = [
            self.r[0], self.r[1], self.r[2],
            self.v[0], self.v[1], self.v[2],
            float(self.mass), float(t_go),
            float(np.linalg.norm(T)),
            float(np.degrees(angle_err)),
        ]
        self.state_pub.publish(state_msg)

        # Landing detection
        if self.r[2] <= 1.0 or (pos_err < 2.0 and vel_err < 1.0):
            self.get_logger().info(
                f'\n{"="*60}'
                f'\n  LANDED  at t = {self.t:.2f} s'
                f'\n  Final position  : {np.round(self.r, 3)}'
                f'\n  Final velocity  : {np.round(self.v, 3)}'
                f'\n  Fuel used       : {m0 - self.mass:.2f} kg'
                f'\n  Pos error       : {pos_err:.3f} m'
                f'\n  Vel error       : {vel_err:.3f} m/s'
                f'\n  Final att error : {np.degrees(angle_err):.2f} deg'
                f'\n  Map coverage    : {coverage:.1f}%'
                f'\n  Barrier steps   : {n_steps}'
                f'\n{"="*60}'
            )
            self.landed = True


# ============================================================
# ENTRY POINT
# ============================================================
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
