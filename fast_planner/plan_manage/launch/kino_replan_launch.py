import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments (defaults taken from original XML)
    ld = LaunchDescription([
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('odom_topic', default_value='/state_ukf/odom'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/pcl_render_node/camera_pose'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth'),
        DeclareLaunchArgument('cloud_topic', default_value='/pcl_render_node/cloud'),
        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),
        DeclareLaunchArgument('max_vel', default_value='3.0'),
        DeclareLaunchArgument('max_acc', default_value='2.0'),
        DeclareLaunchArgument('flight_type', default_value='1'),
        DeclareLaunchArgument('point_num', default_value='2'),
        DeclareLaunchArgument('point0_x', default_value='19.0'),
        DeclareLaunchArgument('point0_y', default_value='0.0'),
        DeclareLaunchArgument('point0_z', default_value='1.0'),
        DeclareLaunchArgument('point1_x', default_value='-19.0'),
        DeclareLaunchArgument('point1_y', default_value='0.0'),
        DeclareLaunchArgument('point1_z', default_value='1.0'),
        DeclareLaunchArgument('point2_x', default_value='0.0'),
        DeclareLaunchArgument('point2_y', default_value='19.0'),
        DeclareLaunchArgument('point2_z', default_value='1.0'),
    ])

    # Paths to included launch files in the same package
    plan_manage_share = get_package_share_directory('plan_manage')
    kino_algorithm_launch = os.path.join(plan_manage_share, 'launch', 'kino_algorithm_launch.py')

    # Include kino_algorithm (forward arguments) using Python launch source
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kino_algorithm_launch),
            launch_arguments={
                'map_size_x_': LaunchConfiguration('map_size_x'),
                'map_size_y_': LaunchConfiguration('map_size_y'),
                'map_size_z_': LaunchConfiguration('map_size_z'),
                'odometry_topic': LaunchConfiguration('odom_topic'),
                'camera_pose_topic': LaunchConfiguration('camera_pose_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'cx': LaunchConfiguration('cx'),
                'cy': LaunchConfiguration('cy'),
                'fx': LaunchConfiguration('fx'),
                'fy': LaunchConfiguration('fy'),
                'max_vel': LaunchConfiguration('max_vel'),
                'max_acc': LaunchConfiguration('max_acc'),
                'flight_type': LaunchConfiguration('flight_type'),
                'point_num': LaunchConfiguration('point_num'),
                'point0_x': LaunchConfiguration('point0_x'),
                'point0_y': LaunchConfiguration('point0_y'),
                'point0_z': LaunchConfiguration('point0_z'),
                'point1_x': LaunchConfiguration('point1_x'),
                'point1_y': LaunchConfiguration('point1_y'),
                'point1_z': LaunchConfiguration('point1_z'),
                'point2_x': LaunchConfiguration('point2_x'),
                'point2_y': LaunchConfiguration('point2_y'),
                'point2_z': LaunchConfiguration('point2_z'),
            }.items()
        )
    )

    # traj_server node (remappings and parameter)
    ld.add_action(
        Node(
            package='plan_manage',
            executable='traj_server',
            name='traj_server',
            output='screen',
            remappings=[
                ('/position_cmd', 'planning/pos_cmd'),
                ('/odom_world', LaunchConfiguration('odom_topic')),
            ],
            parameters=[{'time_forward': 1.5}],
        )
    )

    # waypoint_generator node
    ld.add_action(
        Node(
            package='waypoint_generator',
            executable='waypoint_generator',
            name='waypoint_generator',
            output='screen',
            remappings=[
                ('~odom', LaunchConfiguration('odom_topic')),
                ('~goal', '/move_base_simple/goal'),
                ('~traj_start_trigger', '/traj_start_trigger'),
            ],
            parameters=[{'waypoint_type': 'manual-lonely-waypoint'}],
        )
    )

    return ld
