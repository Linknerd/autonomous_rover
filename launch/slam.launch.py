import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Find where this package's installed files live (urdf/, config/, launch/)
    pkg = get_package_share_directory('autonomous_rover')

    # ── URDF ─────────────────────────────────────────────────────────────────
    # Read the robot description so robot_state_publisher can broadcast TF
    # for every fixed joint (base_footprint→base_link, base_link→laser, etc.)
    urdf_file = os.path.join(pkg, 'urdf', 'john.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── SLAM params ───────────────────────────────────────────────────────────
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([

        # 1. robot_state_publisher
        #    Reads the URDF and publishes TF for every joint.
        #    Comes from autonomous_rover's urdf/john.urdf.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        # 2. scan_relay
        #    The sllidar_ros2 driver publishes scan timestamps ~10x too large
        #    due to a hardware timer bug. This node re-stamps each scan with
        #    the current ROS time and republishes to /scan_corrected so that
        #    SLAM toolbox's TF message filter can match scans to TF transforms.
        Node(
            package='robot_core',
            executable='scan_relay.py',
            name='scan_relay',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # 3. serial_bridge
        #    Reads encoder data from the Arduino over serial, publishes
        #    /odom and the odom→base_footprint TF.
        #    Executable name comes from robot_core's setup.py entry_points.
        # Node(
        #     package='robot_core',
        #     executable='serial_bridge',
        #     name='serial_bridge',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}],
        # ),

        Node(
            package='robot_core',
            executable='serial_bridge_debug',
            name='serial_bridge_debug',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        # 3. SLAM toolbox
        #    Delayed by 5 seconds so serial_bridge has time to connect to
        #    the Arduino and start publishing the odom TF before SLAM
        #    begins processing scans.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('slam_toolbox'),
                            'launch',
                            'online_async_launch.py',
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'slam_params_file': slam_params,
                    }.items(),
                ),
            ],
        ),

    ])
