from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("mavros"),
                    "launch",
                    "px4.launch"
                )
            ),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url")
            }.items()
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="odom_bridge",
            output="screen",
            arguments=[
                "/model/x500_vision_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
            ],
            remappings=[
                ("/model/x500_vision_0/odometry", "/odom")
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="depth_bridge",
            output="screen",
            arguments=[
                "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
            remappings=[
                ("/depth_camera", "/depth")
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="cloud_bridge",
            output="screen",
            arguments=[
                "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            remappings=[
                ("/depth_camera/points", "/cloud")
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="pose_array_bridge",
            output="screen",
            arguments=[
                "/world/baylands/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
            ],
            remappings=[
                ("/world/baylands/pose/info", "/gz_tf_topic")
            ]
        ),

        Node(
            package="utils",
            executable="target_pose_publisher",
            name="target_pose_publisher",
            output="screen",
            parameters=[
                {"target_child_frame": "camera_link"},
                {"input_topic": "/gz_tf_topic"},
                {"output_topic": "/camera_pose"}
            ]
        )
    ])