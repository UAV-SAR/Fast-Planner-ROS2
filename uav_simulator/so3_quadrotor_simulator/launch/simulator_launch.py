import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    so3_control_pkg_share = get_package_share_directory('so3_control')
    so3_quadrotor_simulator_pkg_share = get_package_share_directory('so3_quadrotor_simulator')
    rviz_plugins_pkg_share = get_package_share_directory('rviz_plugins')

    so3_control_config = os.path.join(
        so3_control_pkg_share,
        'config',
        'gains_hummingbird.yaml'
    )
    so3_control_corrections_config = os.path.join(
        so3_control_pkg_share,
        'config',
        'corrections_hummingbird.yaml'
    )
    rviz_config = os.path.join(
        rviz_plugins_pkg_share,
        'config',
        'rviz_config.rviz'
    )

    return LaunchDescription([
        Node(
            package='so3_quadrotor_simulator',
            executable='quadrotor_simulator_so3',
            name='quadrotor_simulator_so3',
            output='screen',
            parameters=[
                {'rate.odom': 100.0},
                {'simulator.init_state_x': -5.0},
                {'simulator.init_state_y': 0.0},
                {'simulator.init_state_z': 3.0}
            ],
            remappings=[
                ('~/odom', '/visual_slam/odom'),
                ('~/cmd', 'so3_cmd'),
                ('~/imu', 'sim/imu'),
                ('~/force_disturbance', 'force_disturbance'),
                ('~/moment_disturbance', 'moment_disturbance')
            ]
        ),

        # In ROS1, this was a nodelet. In ROS2, the equivalent is a composable node.
        # For a simple migration, we assume it has been ported as a regular standalone node.
        # The executable name might need to be adjusted based on how it was ported.
        Node(
            package='so3_control',
            executable='so3_control_nodelet',
            name='so3_control',
            output='screen',
            parameters=[
                so3_control_config,
                so3_control_corrections_config,
                {'mass': 0.98},
                {'use_angle_corrections': False},
                {'use_external_yaw': False},
                {'gains.rot.z': 1.0},
                {'gains.ang.z': 0.1}
            ],
            remappings=[
                ('~/odom', '/state_ukf/odom'),
                ('~/position_cmd', 'position_cmd'),
                ('~/motors', 'motors'),
                ('~/corrections', 'corrections'),
                ('~/so3_cmd', 'so3_cmd'),
                ('~/imu', 'sim/imu')
            ]
        ),

        Node(
            package='so3_disturbance_generator',
            executable='so3_disturbance_generator',
            name='so3_disturbance_generator',
            output='screen',
            remappings=[
                ('~/odom', '/visual_slam/odom'),
                ('~/noisy_odom', '/state_ukf/odom'),
                ('~/correction', '/visual_slam/correction'),
                ('~/force_disturbance', 'force_disturbance'),
                ('~/moment_disturbance', 'moment_disturbance')
            ]
        ),

        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name='odom_visualization_ukf',
            output='screen',
            parameters=[
                {'color.a': 0.8},
                {'color.r': 1.0},
                {'color.g': 0.0},
                {'color.b': 0.0},
                {'covariance_scale': 100.0}
            ],
            remappings=[
                ('~/odom', '/visual_slam/odom')
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
