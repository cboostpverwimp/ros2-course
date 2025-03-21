#!/usr/bin/env python3
"""
:module:        gz_moveit_full.launch.py
:description:   Template for GZ-sim robot controller launch
:owner:         (C) C-Boost B.V. (cboost) - All Rights Reserved
:author:        [Patrick Verwimp](mailto:patrick.verwimp@cboost.nl)
:project:       Cboost Internal

This file is proprietary and confidential.
Unauthorized copying of this file via any medium is strictly prohibited.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer

__author__ = "Patrick Verwimp"
__copyright__ = "(C) C-Boost B.V. (cboost) - All Rights Reserved"
__credits__ = ["Patrick Verwimp", "Simon Mingaars"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Patrick Verwimp"
__email__ = "patrick.verwimp@cboost.nl"
__status__ = "Prototype"

def generate_launch_description():
#region LaunchConfiguration

    launch_as_standalone_node = LaunchConfiguration("launch_as_standalone_node")
    use_rviz = LaunchConfiguration("use_rviz")

    stand_alone_arg = DeclareLaunchArgument(
        "launch_as_standalone_node",
        default_value="False"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False"
    )

#endregion
#region Parameters
    # PARTS TO EDIT -----------------------------------------------------------
    mapping = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "",
        "dof": "6",
    }
    prefix = "motoman_"
    robot_name = "hc10dtp"
    rviz_config_file = "servo.rviz"
    # -------------------------------------------------------------------------

    urdf_file = f"{prefix}{robot_name}.urdf.xacro"
    ros2_controller_file = "ros2_controllers.yaml"
    
    gazebo_pkg_share = get_package_share_directory(f"{robot_name}_gzsim")
    moveit_pkg_share = get_package_share_directory(f"{robot_name}_moveit_config")

    # Conditioned vaiables
    use_sim_time = {"use_sim_time": True}

    robot_description_path = os.path.join(gazebo_pkg_share, "config", urdf_file)
    rviz_config_path = os.path.join(moveit_pkg_share, "config", rviz_config_file)
    ros2_controllers_path = os.path.join(moveit_pkg_share, "config", ros2_controller_file)

    # Create configuration for move_group
    moveit_config = (
        MoveItConfigsBuilder(robot_name= robot_name, package_name= f"{robot_name}_moveit_config")
        .robot_description(
            file_path= robot_description_path,
            mappings= mapping)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description= True,
            publish_robot_description_semantic= True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder(f"{robot_name}_moveit_config")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}

#endregion
#region Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            use_sim_time
        ],
        condition=IfCondition(use_rviz)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{robot_name}_controller", "-c", "/controller_manager"],
    )
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            moveit_config.to_dict(),
            use_sim_time,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )
#endregion
#region NodeContainer
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    moveit_config.to_dict(),
                    use_sim_time,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[use_sim_time, moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "base_link", "frame_id": "world"}],
            ),
        ],
        output="screen",
    )
#endregion
#region LaunchDescription
    return LaunchDescription(
        [
            use_rviz_arg,
            stand_alone_arg,
            rviz_node,
            container,
            servo_node,
            move_group_node,
            joint_state_broadcaster_spawner,
            controller_spawner,
        ]
    )
#endregion