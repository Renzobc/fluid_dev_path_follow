import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Path to rviz configuration
    # Path to the package

    pkg_share = FindPackageShare(package='path_follow').find('path_follow')

    default_rviz_config_path = os.path.join(pkg_share, 'config', 'rviz.rviz')

    config_initial_pose_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # Load the parameters for a ComposableNode
    with open(config_initial_pose_path, 'r') as file:
        initial_pose_param = yaml.safe_load(
            file)['/Robot_Link_Initial_Pose']['ros__parameters']

    # Load the static transforms
    with open(config_initial_pose_path, 'r') as file:
        frames = yaml.safe_load(file)['static_transforms']
        framesA=frames['point_A']
        framesB=frames['point_B']
        framesC=frames['point_C']
        framesD=frames['point_D']
        framesE=frames['point_E']

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to rviz config file'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_initial_pose)

    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=framesA                         #['2', '0', '1', '1.57', '0', '0', 'world', 'A']
        ))
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=framesB                        #['2', '0', '1', '1.57', '0', '0', 'A', 'B']
        ))
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=framesC                        #['2', '0', '1', '1.57', '0', '0', 'B', 'C']
        ))
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=framesD                               #['2', '0', '1', '1.57', '0', '0', 'C', 'D']
        ))
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=framesE                              #['2', '0', '1', '1.57', '0', '0', 'D', 'E']
        ))

    # start launch of rviz
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ))

    container = ComposableNodeContainer(
        name='robot_link_path_follow_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='path_follow',
                plugin='path_follow::RobotControl',
                name='RobotLinkControl'
            ),
            ComposableNode(
                package='path_follow',
                plugin='path_follow::RobotLink',
                name='RobotLinkModel'
            ),
        ],
        output='screen',
    )

    container_init = ComposableNodeContainer(
        name='robot_link_init',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='path_follow',
                plugin='path_follow::publish_path',
                name='publish_path'
            ),
            ComposableNode(
                package='path_follow',
                plugin='path_follow::InitialPoseBroadcast',
                name='Robot_Link_Initial_Pose',
                parameters=[initial_pose_param]
            ),
        ],
        output='screen',
    )

    ld.add_action(container_init)
    ld.add_action(container)

    return ld
