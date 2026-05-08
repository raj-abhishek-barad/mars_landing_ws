"""
Lander Localisation Node
=========================
Subscribes to EKF output and displays position + velocity.
Also publishes compact state vector for logging.

Topics:
  IN:  /odometry/filtered  (nav_msgs/Odometry)
  OUT: /lander/state       (Float64MultiArray) [x,y,z,vx,vy,vz]
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np


class LanderLocalisation(Node):
    def __init__(self):
        super().__init__('lander_localisation')

        self.state_pub = self.create_publisher(
            Float64MultiArray, '/lander/state', 10)

        self.create_subscription(
            Odometry, '/odometry/filtered', self.callback, 10)

        self.get_logger().info('Lander localisation node started.')

    def callback(self, msg):
        p   = msg.pose.pose.position
        v   = msg.twist.twist.linear
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        pos_std = np.sqrt(np.maximum(np.diag(cov[:3, :3]), 0))

        self.get_logger().info(
            f'\n--- Lander State ---'
            f'\nPosition : x={p.x:9.2f}m  y={p.y:9.2f}m  z={p.z:9.2f}m'
            f'\nVelocity : vx={v.x:7.3f}  vy={v.y:7.3f}  vz={v.z:7.3f} m/s'
            f'\nUncert 1σ: σx={pos_std[0]:.3f}  σy={pos_std[1]:.3f}  '
            f'σz={pos_std[2]:.3f} m'
            f'\nSpeed    : {np.linalg.norm([v.x,v.y,v.z]):.3f} m/s'
        )

        state_msg = Float64MultiArray()
        state_msg.data = [p.x, p.y, p.z, v.x, v.y, v.z]
        self.state_pub.publish(state_msg)


def main():
    rclpy.init()
    rclpy.spin(LanderLocalisation())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
