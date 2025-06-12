#!/usr/bin/env python3
#Got the necessity of the line above from an error in the terminal.

import rospy #initialize ROS with python
import sys
import copy
from math import pi
import moveit_commander  #Central controller of robot's movement commands (that come from trajectory).
import moveit_msgs.msg
import yaml #import YAML library for saving data
#Yaml file is recommended in the MoveIt documentation, as the functions output is automatically configured to it I think?
import pickle #Pickle is my new best friend. all my homies hate yaml.
#from moveit_commander.move_group import MoveGroupCommander  
from geometry_msgs.msg import Pose  #Pose object message that contain robot's configuration. Defined in the documentation.   

def synchronize_robot_to_trajectory(group, trajectory):
    # Extract the joint positions from the trajectory's first point
    start_joint_positions = trajectory.joint_trajectory.points[0].positions
    group.set_joint_value_target(start_joint_positions)
    rospy.loginfo("Synchronizing robot to start state of trajectory...")
    if not group.go(wait=True):
        rospy.logerr("Failed to move robot to trajectory start state. Aborting.")
        return False
    rospy.sleep(1)  # Allow time for stabilization
    return True

def load_trajectories():
    rospy.init_node('load_trajectories', anonymous=True)  #Start ROS node for this script.
    #moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.roscpp_initialize(['/home/potato/catkin_ws/src/hw_pkg/src/collect_trajectories_script.py', '__ns:=/my_gen3/'])
    #Initialize RobotCommander and MoveGroupCommander using the namespace and arm group
    #Create interfaces to the robot and planning scene
    robot = moveit_commander.RobotCommander("robot_description")
    scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    group = moveit_commander.MoveGroupCommander("arm", ns=rospy.get_namespace()) # Initialize MoveGroupCommander with the arm group name
    start_pose = group.get_current_pose().pose  #Start pose is the pose the robot starts at.
    #load trajectories from YAML file
    with open('/home/potato/catkin_ws/src/hw_pkg/trajectories.pkl', 'rb') as file:
        trajectories = pickle.load(file)


    # Execute trajectories
    for i, trajectory in enumerate(trajectories):
        rospy.loginfo(f"Executing trajectory {i + 1}...")
        if not synchronize_robot_to_trajectory(group, trajectory):
            rospy.logerr(f"Skipping trajectory {i + 1} due to synchronization failure.")
            continue
        try:
            group.execute(trajectory, wait=True)
        except Exception as e:
            rospy.logerr(f"Execution failed for trajectory {i + 1}: {e}")
        rospy.sleep(1)

    rospy.loginfo("Trajectory execution process completed.")


    #execute each trajectory in sequence
    # for trajectory in trajectories: #loop through each saved trajectory
    #     group.execute(trajectory,wait=True) #Execute each trajectory (which is a plan object) with waiting.
    #     group.set_pose_target(start_pose)
    #     group.go(wait=True)

if __name__=='__main__':
    rospy.init_node('load_trajectories', anonymous=True)  #Start the ROS node for this script.
    load_trajectories()  #Run the function to load and execute.
