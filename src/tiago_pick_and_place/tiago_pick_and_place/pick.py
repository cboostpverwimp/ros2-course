#!/usr/bin/env python3
"""
Code snippet commanding MoveIt planning and execution on Tiago from Python

# Reminder of available MoveItPy functions:
#   print(dir(robot_state)) # 'clear_attached_bodies', 'dirty', 'get_frame_transform', 'get_global_link_transform', 'get_jacobian', 'get_joint_group_accelerations', 'get_joint_group_positions', 'get_joint_group_velocities', 'get_pose', 'joint_accelerations', 'joint_efforts', 'joint_positions', 'joint_velocities', 'robot_model', 'set_from_ik', 'set_joint_group_accelerations', 'set_joint_group_active_positions', 'set_joint_group_positions', 'set_joint_group_velocities', 'set_to_default_values', 'set_to_random_positions', 'state_info', 'state_tree', 'update'
#   print(dir(tiago_arm))   # 'get_named_target_state_values', 'get_start_state', 'named_target_states', 'plan', 'planning_group_name', 'set_goal_state', 'set_path_constraints', 'set_start_state', 'set_start_state_to_current_state', 'set_workspace', 'unset_workspace'
#   print(dir(tiago))       # 'execute', 'get_planning_component', 'get_planning_scene_monitor', 'get_robot_model', 'get_trajectory_execution_manager', 'shutdown'
#   print(dir(plan_result)) # 'error_code', 'planner_id', 'planning_time', 'start_state', 'trajectory'
"""
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    logger = get_logger("moveit_py_pick")
    tiago = MoveItPy(node_name="moveit_py_pick")
    robot_model = tiago.get_robot_model()
    tiago_arm = tiago.get_planning_component("arm")
    logger.info("moveit_py_pick node successfully connected to Tiago!")

    ###########################################################################
    # Plan 1 - set goal state with RobotState object (using its joint states)
    ###########################################################################
    """
    # Create a state from the current robot pose. Remember current state for later
    robot_initial_state = RobotState(robot_model)

    # Set another robot_state to default values = Tiago's arm extended
    robot_state = RobotState(robot_model)
    robot_state.set_to_default_values()
    
    # Set plan start state to current state
    tiago_arm.set_start_state_to_current_state()

    # Set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    tiago_arm.set_goal_state(robot_state=robot_state)
    
    # Plan trajectory to the goal state
    plan_result = tiago_arm.plan()

    # If it worked, execute the planned motion
    if plan_result:
        logger.info("Executing plan")
        tiago.execute(plan_result.trajectory, controllers=[])
    else:
        logger.error("Planning failed")
    
    # Go back to the initial state saved before
    tiago_arm.set_start_state_to_current_state()
    tiago_arm.set_goal_state(robot_state=robot_initial_state)
    plan_result = tiago_arm.plan()
    if plan_result:
        logger.warn("Moving arm to the joints goal (RobotState goal)...")
        tiago.execute(plan_result.trajectory, controllers=[])
    """
    ###########################################################################
    # Plan 2 - set cartesian goal for the gripper with PoseStamped message
    ###########################################################################
    """
    # Set plan start state to current state
    tiago_arm.set_start_state_to_current_state()

    # Create pose goal with PoseStamped message
    pose_goal = PoseStamped()

    # The frame for the following cordinates...
    pose_goal.header.frame_id = "base_link"

    # ... the target end effector position...
    pose_goal.pose.position.x = 0.7
    pose_goal.pose.position.y = 0.1
    pose_goal.pose.position.z = 0.4

    # ... and the target end effector quaternion...
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 1.0
    # ...you can get these coordinates from the current robot pose with `ros2 run tf2_ros tf2_echo base_link gripper_grasping_frame`

    tiago_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="gripper_grasping_frame")

    # Plan to the cartesian goal
    plan_result = tiago_arm.plan()

    # If planning worked, execute the planned motion
    if plan_result:
        logger.warn("Moving arm to the cartesian goal...")
        tiago.execute(plan_result.trajectory, controllers=[])
    """

    ###########################################################################
    # Plan 3 - Open and close gripper
    ###########################################################################
    """
    tiago_gripper = tiago.get_planning_component("gripper")

    # Gripper opening
    gripper_open = RobotState(robot_model)
    gripper_open_joints = {"left_finger": 0.04, "right_finger": 0.04}
    gripper_open.set_joint_group_positions("gripper", list(gripper_open_joints.values()))
    tiago_gripper.set_goal_state(robot_state=gripper_open)

    gripper_opening_result = tiago_gripper.plan()
    if gripper_opening_result:
        logger.warn("Opening Tiago's gripper...")
        tiago.execute(gripper_opening_result.trajectory, controllers=[])

    # Gripper closing
    gripper_closed = RobotState(robot_model)
    gripper_closed_joints = {"left_finger": 0.0, "right_finger": 0.0}
    gripper_closed.set_joint_group_positions("gripper", list(gripper_closed_joints.values()))
    tiago_gripper.set_goal_state(robot_state=gripper_closed)

    gripper_closing_result = tiago_gripper.plan()
    if gripper_closing_result:
        logger.warn("Closing Tiago's gripper...")
        tiago.execute(gripper_closing_result.trajectory, controllers=[])
    """