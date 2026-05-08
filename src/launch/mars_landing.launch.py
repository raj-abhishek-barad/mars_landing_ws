"""
Mars Landing Launch File
=========================
Launches the full simulation stack in the correct order:
  1. Gazebo server (headless physics)
  2. Gazebo GUI
  3. ROS-Gazebo bridge (odometry + wrench + sensors)
  4. EKF localisation
  5. Perception node (altimeter + camera → terrain map)
  6. Method C guidance node
  7. Lander localisation display

Usage:
  ros2 launch src/launch/mars_landing.launch.py
  ros2 launch src/launch/mars_landing.launch.py headless:=true
  ros2 launch src/launch/mars_landing.launch.py case:=1
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, LogInfo)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os


WORLD_FILE = os.path.expanduser(
    '~/ros2_ws/src/my_worlds/worlds/mars_terrain.world')
EKF_CONFIG = os.path.join(
    os.path.dirname(__file__), '../../config/ekf.yaml')


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI')

    case_arg = DeclareLaunchArgument(
        'case', default_value='3',
        description='Initial condition case (1, 2, or 3)')

    # ── Gazebo Server ─────────────────────────────────────────
    gz_server = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && '
            f'export GZ_SIM_RESOURCE_PATH=~/.gz/models && '
            f'gz sim -s {WORLD_FILE} --render-engine ogre -v 4'
        ],
        output='screen',
        name='gz_server'
    )

    # ── Gazebo GUI ────────────────────────────────────────────
    gz_gui = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/jazzy/setup.bash && '
             'gz sim -g --render-engine ogre'],
        output='screen',
        name='gz_gui',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # ── Unpause simulation after 3s ───────────────────────────
    unpause = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'gz service -s /world/mars_world/control '
                 '--reqtype gz.msgs.WorldControl '
                 '--reptype gz.msgs.Boolean '
                 '--timeout 2000 '
                 "--req 'pause: false'"],
            output='screen'
        )]
    )

    # ── ROS-Gazebo Bridge ─────────────────────────────────────
    bridge = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && '
                'ros2 run ros_gz_bridge parameter_bridge '
                '/model/lander/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry '
                '/altimeter/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan '
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image '
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU '
                '/lander/thrust@geometry_msgs/msg/Wrench]gz.msgs.Wrench'
            ],
            output='screen',
            name='ros_gz_bridge'
        )]
    )

    # ── EKF Localisation ──────────────────────────────────────
    ekf_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[EKF_CONFIG]
        )]
    )

    # ── Perception Node ───────────────────────────────────────
    perception_node = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && '
                'python3 src/perception/perception_node.py'
            ],
            output='screen',
            name='perception_node'
        )]
    )

    # ── Method C Guidance ─────────────────────────────────────
    guidance_node = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && '
                'python3 src/guidance/method_C_guidance_node.py'
            ],
            output='screen',
            name='guidance_node'
        )]
    )

    # ── Localisation Display ──────────────────────────────────
    localisation_node = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && '
                'python3 src/localisation/lander_localisation.py'
            ],
            output='screen',
            name='localisation_node'
        )]
    )

    return LaunchDescription([
        headless_arg,
        case_arg,
        LogInfo(msg='--- Mars Landing Simulation Starting ---'),
        gz_server,
        gz_gui,
        unpause,
        bridge,
        ekf_node,
        perception_node,
        guidance_node,
        localisation_node,
        LogInfo(msg='--- All nodes launched ---'),
    ])
