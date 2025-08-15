import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument('map_size_x_'),
        DeclareLaunchArgument('map_size_y_'),
        DeclareLaunchArgument('map_size_z_'),

        DeclareLaunchArgument('odometry_topic'),
        DeclareLaunchArgument('camera_pose_topic'),
        DeclareLaunchArgument('depth_topic'),
        DeclareLaunchArgument('cloud_topic'),

        DeclareLaunchArgument('cx'),
        DeclareLaunchArgument('cy'),
        DeclareLaunchArgument('fx'),
        DeclareLaunchArgument('fy'),

        DeclareLaunchArgument('max_vel'),
        DeclareLaunchArgument('max_acc'),

        DeclareLaunchArgument('point_num'),
        DeclareLaunchArgument('point0_x'),
        DeclareLaunchArgument('point0_y'),
        DeclareLaunchArgument('point0_z'),
        DeclareLaunchArgument('point1_x'),
        DeclareLaunchArgument('point1_y'),
        DeclareLaunchArgument('point1_z'),
        DeclareLaunchArgument('point2_x'),
        DeclareLaunchArgument('point2_y'),
        DeclareLaunchArgument('point2_z'),

        DeclareLaunchArgument('flight_type'),
    ])

    fast_planner_params = {
        'planner_node': {'planner': 1},

        'fsm': {
            'flight_type': LaunchConfiguration('flight_type'),
            'thresh_replan': 1.5,
            'thresh_no_replan': 2.0,
            'waypoint_num': LaunchConfiguration('point_num'),
            'waypoint0_x': LaunchConfiguration('point0_x'),
            'waypoint0_y': LaunchConfiguration('point0_y'),
            'waypoint0_z': LaunchConfiguration('point0_z'),
            'waypoint1_x': LaunchConfiguration('point1_x'),
            'waypoint1_y': LaunchConfiguration('point1_y'),
            'waypoint1_z': LaunchConfiguration('point1_z'),
            'waypoint2_x': LaunchConfiguration('point2_x'),
            'waypoint2_y': LaunchConfiguration('point2_y'),
            'waypoint2_z': LaunchConfiguration('point2_z'),
        },

        'sdf_map': {
            'resolution': 0.1,
            'map_size_x': LaunchConfiguration('map_size_x_'),
            'map_size_y': LaunchConfiguration('map_size_y_'),
            'map_size_z': LaunchConfiguration('map_size_z_'),
            'local_update_range_x': 5.5,
            'local_update_range_y': 5.5,
            'local_update_range_z': 4.5,
            'obstacles_inflation': 0.099,
            'local_bound_inflate': 0.0,
            'local_map_margin': 50,
            'ground_height': -1.0,
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'use_depth_filter': True,
            'depth_filter_tolerance': 0.15,
            'depth_filter_maxdist': 5.0,
            'depth_filter_mindist': 0.2,
            'depth_filter_margin': 2,
            'k_depth_scaling_factor': 1000.0,
            'skip_pixel': 2,
            'p_hit': 0.65,
            'p_miss': 0.35,
            'p_min': 0.12,
            'p_max': 0.90,
            'p_occ': 0.80,
            'min_ray_length': 0.5,
            'max_ray_length': 4.5,
            'esdf_slice_height': 0.3,
            'visualization_truncate_height': 2.49,
            'virtual_ceil_height': 2.5,
            'show_occ_time': False,
            'show_esdf_time': False,
            'pose_type': 1,
            'frame_id': 'world',
        },

        'manager': {
            'max_vel': LaunchConfiguration('max_vel'),
            'max_acc': LaunchConfiguration('max_acc'),
            'max_jerk': 4.0,
            'dynamic_environment': 0,
            'local_segment_length': 6.0,
            'clearance_threshold': 0.2,
            'control_points_distance': 0.5,
            'use_geometric_path': False,
            'use_kinodynamic_path': True,
            'use_topo_path': False,
            'use_optimization': True,
        },

        'search': {
            'max_tau': 0.6,
            'init_max_tau': 0.8,
            'max_vel': LaunchConfiguration('max_vel'),
            'max_acc': LaunchConfiguration('max_acc'),
            'w_time': 10.0,
            'horizon': 7.0,
            'lambda_heu': 5.0,
            'resolution_astar': 0.1,
            'time_resolution': 0.8,
            'margin': 0.2,
            'allocate_num': 100000,
            'check_num': 5,
        },

        'optimization': {
            'lambda1': 10.0,
            'lambda2': 5.0,
            'lambda3': 0.00001,
            'lambda4': 0.01,
            'lambda7': 100.0,
            'dist0': 0.4,
            'max_vel': LaunchConfiguration('max_vel'),
            'max_acc': LaunchConfiguration('max_acc'),
            'algorithm1': 15,
            'algorithm2': 11,
            'max_iteration_num1': 2,
            'max_iteration_num2': 300,
            'max_iteration_num3': 200,
            'max_iteration_num4': 200,
            'max_iteration_time1': 0.0001,
            'max_iteration_time2': 0.005,
            'max_iteration_time3': 0.003,
            'max_iteration_time4': 0.003,
            'order': 3,
        },

        'bspline': {
            'limit_vel': LaunchConfiguration('max_vel'),
            'limit_acc': LaunchConfiguration('max_acc'),
            'limit_ratio': 1.1,
        },
    }

    fast_planner_node = Node(
        package='plan_manage',
        executable='fast_planner_node',
        name='fast_planner_node',
        output='screen',
        remappings=[
            ('/odom_world', LaunchConfiguration('odometry_topic')),
            ('/sdf_map/odom', LaunchConfiguration('odometry_topic')),
            ('/sdf_map/cloud', LaunchConfiguration('cloud_topic')),
            ('/sdf_map/pose', LaunchConfiguration('camera_pose_topic')),
            ('/sdf_map/depth', LaunchConfiguration('depth_topic')),
        ],
        parameters=[fast_planner_params]
    )

    ld.add_action(fast_planner_node)
    return ld