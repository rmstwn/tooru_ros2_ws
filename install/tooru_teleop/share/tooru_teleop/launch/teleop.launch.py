import os
import yaml
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="tooru_teleop_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="tooru_teleop",
                plugin="tooru_teleop::JoyToTwistPub",
                name="controller_to_twist_node",
            ),
            launch_ros.descriptions.ComposableNode(
                package="tooru_mobile",
                plugin="tooru_mobile::TwistToJointPub",
                name="twist_to_joint_node",
            ),
            launch_ros.descriptions.ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    dynamixel_node = launch_ros.actions.Node(
        package="tooru_dynamixel",
        executable="dynamixel_node",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            dynamixel_node,
            container,
        ]
    )
