from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_size_x", default_value="40.0"),
        DeclareLaunchArgument("map_size_y", default_value="20.0"),
        DeclareLaunchArgument("map_size_z", default_value="5.0"),

        DeclareLaunchArgument("odom_topic", default_value="/state_ukf/odom"),
        DeclareLaunchArgument("camera_pose_topic", default_value="/pcl_render_node/camera_pose"),
        DeclareLaunchArgument("depth_topic", default_value="/pcl_render_node/depth"),
        DeclareLaunchArgument("cloud_topic", default_value="/pcl_render_node/cloud"),

        # intrinsic params of the depth camera
        DeclareLaunchArgument("cx", default_value="321.04638671875"),
        DeclareLaunchArgument("cy", default_value="243.44969177246094"),
        DeclareLaunchArgument("fx", default_value="387.229248046875"),
        DeclareLaunchArgument("fy", default_value="387.229248046875"),

        # maximum velocity and acceleration the drone will reach
        DeclareLaunchArgument("max_vel", default_value="3.0"),
        DeclareLaunchArgument("max_acc", default_value="2.5"),

        # 1: use 2D Nav Goal to select goal
        # 2: use global waypoints below
        # 3: use global waypoints below to set reference path
        DeclareLaunchArgument("flight_type", default_value="2"),

        # global waypoints
        DeclareLaunchArgument("point_num", default_value="3"),

        DeclareLaunchArgument("point0_x", default_value="19.0"),
        DeclareLaunchArgument("point0_y", default_value="0.0"),
        DeclareLaunchArgument("point0_z", default_value="1.0"),

        # set more waypoints if you need
        DeclareLaunchArgument("point1_x", default_value="19.0"),
        DeclareLaunchArgument("point1_y", default_value="10.0"),
        DeclareLaunchArgument("point1_z", default_value="1.0"),

        DeclareLaunchArgument("point2_x", default_value="-19.0"),
        DeclareLaunchArgument("point2_y", default_value="0.0"),
        DeclareLaunchArgument("point2_z", default_value="1.0"),

        # planner manager node
        Node(
            package="plan_manage",
            executable="fast_planner_node",
            name="fast_planner_node",
            output="screen",
            remappings=[
                ("/odom_world", LaunchConfiguration("odom_topic")),
                ("/sdf_map/odom", LaunchConfiguration("odom_topic")),
                ("/sdf_map/cloud", LaunchConfiguration("cloud_topic")),
                ("/sdf_map/pose", LaunchConfiguration("camera_pose_topic")),
                ("/sdf_map/depth", LaunchConfiguration("depth_topic")),
            ],
            parameters=[
                # replanning method
                { "planner_node/planner": 2 },

                # planning fsm
                {
                    "fsm/flight_type": ParameterValue(LaunchConfiguration("flight_type"), value_type=int),
                    "fsm/thresh_replan": 0.5,
                    "fsm/thresh_no_replan": 2.0,

                    "fsm/waypoint_num": ParameterValue(LaunchConfiguration("point_num"), value_type=int),
                    "fsm/waypoint0_x": ParameterValue(LaunchConfiguration("point0_x"), value_type=float),
                    "fsm/waypoint0_y": ParameterValue(LaunchConfiguration("point0_y"), value_type=float),
                    "fsm/waypoint0_z": ParameterValue(LaunchConfiguration("point0_z"), value_type=float),
                    "fsm/waypoint1_x": ParameterValue(LaunchConfiguration("point1_x"), value_type=float),
                    "fsm/waypoint1_y": ParameterValue(LaunchConfiguration("point1_y"), value_type=float),
                    "fsm/waypoint1_z": ParameterValue(LaunchConfiguration("point1_z"), value_type=float),
                    "fsm/waypoint2_x": ParameterValue(LaunchConfiguration("point2_x"), value_type=float),
                    "fsm/waypoint2_y": ParameterValue(LaunchConfiguration("point2_y"), value_type=float),
                    "fsm/waypoint2_z": ParameterValue(LaunchConfiguration("point2_z"), value_type=float),
                },

                # sdf map
                {
                    "sdf_map/resolution": 0.1,
                    "sdf_map/map_size_x": ParameterValue(LaunchConfiguration("map_size_x"), value_type=float),
                    "sdf_map/map_size_y": ParameterValue(LaunchConfiguration("map_size_y"), value_type=float),
                    "sdf_map/map_size_z": ParameterValue(LaunchConfiguration("map_size_z"), value_type=float),
                    "sdf_map/local_update_range_x": 5.5,
                    "sdf_map/local_update_range_y": 5.5,
                    "sdf_map/local_update_range_z": 4.5,
                    "sdf_map/obstacles_inflation": 0.099,
                    "sdf_map/local_bound_inflate": 0.5,
                    "sdf_map/local_map_margin": 50,
                    "sdf_map/ground_height": -1.0,

                    # camera parameter
                    "sdf_map/cx": ParameterValue(LaunchConfiguration("cx"), value_type=float),
                    "sdf_map/cy": ParameterValue(LaunchConfiguration("cy"), value_type=float),
                    "sdf_map/fx": ParameterValue(LaunchConfiguration("fx"), value_type=float),
                    "sdf_map/fy": ParameterValue(LaunchConfiguration("fy"), value_type=float),

                    # depth filter
                    "sdf_map/use_depth_filter": True,
                    "sdf_map/depth_filter_tolerance": 0.15,
                    "sdf_map/depth_filter_maxdist": 4.5,
                    "sdf_map/depth_filter_mindist": 0.2,
                    "sdf_map/depth_filter_margin": 2,
                    "sdf_map/k_depth_scaling_factor": 1000.0,
                    "sdf_map/skip_pixel": 3,

                    # local fusion
                    "sdf_map/p_hit": 0.65,
                    "sdf_map/p_miss": 0.35,
                    "sdf_map/p_min": 0.12,
                    "sdf_map/p_max": 0.90,
                    "sdf_map/p_occ": 0.80,
                    "sdf_map/min_ray_length": 0.5,
                    "sdf_map/max_ray_length": 4.5,

                    "sdf_map/esdf_slice_height": 0.3,
                    "sdf_map/visualization_truncate_height": 2.5,
                    "sdf_map/virtual_ceil_height": 3.0,
                    "sdf_map/show_occ_time": False,
                    "sdf_map/show_esdf_time": False,
                    "sdf_map/pose_type": 1,
                    "sdf_map/frame_id": "world"
                },

                # planner manager
                {
                    "manager/max_vel": ParameterValue(LaunchConfiguration("max_vel"), value_type=float),
                    "manager/max_acc": ParameterValue(LaunchConfiguration("max_acc"), value_type=float),
                    "manager/max_jerk": 4.0,
                    "manager/dynamic_environment": 0,
                    "manager/local_segment_length": 7.0,
                    "manager/clearance_threshold": 0.2,
                    "manager/control_points_distance": 0.3,

                    "manager/use_geometric_path": False,
                    "manager/use_kinodynamic_path": False,
                    "manager/use_topo_path": True,
                    "manager/use_optimization": True
                },

                # topology path finding
                {
                    "topo_prm/sample_inflate_x": 1.0,
                    "topo_prm/sample_inflate_y": 3.5,
                    "topo_prm/sample_inflate_z": 1.0,
                    "topo_prm/clearance": 0.3,
                    "topo_prm/max_sample_time": 0.005,
                    "topo_prm/max_sample_num": 2000,
                    "topo_prm/max_raw_path": 300,
                    "topo_prm/max_raw_path2": 25,
                    "topo_prm/short_cut_num": 1,
                    "topo_prm/reserve_num": 6,
                    "topo_prm/ratio_to_short": 5.5,
                    "topo_prm/parallel_shortcut": True
                },

                # trajectory optimization
                {
                    "optimization/lambda1": 10.0,
                    "optimization/lambda2": 5.0,
                    "optimization/lambda3": 0.0000,
                    "optimization/lambda4": 0.001,
                    "optimization/lambda5": 1.5,
                    "optimization/lambda6": 10.0,
                    "optimization/lambda7": 20.0,

                    "optimization/dist0": 0.4,
                    "optimization/dist1": 0.0,
                    "optimization/max_vel": ParameterValue(LaunchConfiguration("max_vel"), value_type=float),
                    "optimization/max_acc": ParameterValue(LaunchConfiguration("max_acc"), value_type=float),

                    "optimization/algorithm1": 15,
                    "optimization/algorithm2": 11,
                    "optimization/max_iteration_num1": 2,
                    "optimization/max_iteration_num2": 300,
                    "optimization/max_iteration_time1": 0.0001,
                    "optimization/max_iteration_time2": 0.005,
                    "optimization/order": 3,

                    "bspline/limit_vel": ParameterValue(LaunchConfiguration("max_vel"), value_type=float),
                    "bspline/limit_acc": ParameterValue(LaunchConfiguration("max_acc"), value_type=float),
                    "bspline/limit_ratio": 1.1,

                    "heading_planner/yaw_diff": 5 * 3.1415926 / 180.0,
                    "heading_planner/half_vert_num": 3,
                    "heading_planner/lambda1": 2.0,
                    "heading_planner/lambda2": 1.0,
                    "heading_planner/max_yaw_rate": 60 * 3.1415926 / 180.0,
                    "heading_planner/w": 10.0,
                    "fsm/act_map": False
                }
            ]
        ),

        # trajectory server node
        Node(
            package="plan_manage",
            executable="traj_server",
            name="traj_server",
            output="screen",
            remappings=[
                ("/position_cmd", "planning/pos_cmd"),
                ("/odom_world", LaunchConfiguration("odom_topic")),
            ],
            parameters=[
                { "traj_server/time_forward": 1.5 }
            ]
        ),

        # waypoint generator node
        Node(
            package="waypoint_generator",
            executable="waypoint_generator",
            name="waypoint_generator",
            output="screen",
            remappings=[
                ("odom", LaunchConfiguration("odom_topic")),        
                ("goal", "/move_base_simple/goal"),
                ("traj_start_trigger", "/traj_start_trigger")
            ],
            parameters=[
                { "waypoint_type": "manual-lonely-waypoint" }
            ]
        )
    ])
