#!/usr/bin/env python3
#Got the necessity of the line above from an error in the terminal.

import rospy #initialize ROS with python
import sys
import copy
from math import pi
import random
import moveit_commander  #Central controller of robot's movement commands (that come from trajectory).
import moveit_msgs.msg
import yaml #import YAML library for saving data
#Yaml file is recommended in the MoveIt documentation, as the functions output is automatically configured to it I think? Turns out not.
import pickle #Pickle is my new best friend. all my homies hate yaml.
#from moveit_commander.move_group import MoveGroupCommander  
from geometry_msgs.msg import Pose  #Pose object message that contain robot's configuration. Defined in the documentation.    

def reach_neutral_pose(group): 
    # neutral_pose = group.get_current_pose().pose
    # neutral_pose.position.x = 0.0  # Example neutral x position (meters)
    # neutral_pose.position.y = 0.0  # Centered in the y direction
    # neutral_pose.position.z = 0.0  # Neutral height above the base (meters)
    # neutral_pose.orientation.x = 0.0  # Aligned orientation
    # neutral_pose.orientation.y = 0.0
    # neutral_pose.orientation.z = 0.0
    # neutral_pose.orientation.w = 1.0  # Quaternion representation of no rotation
    # group.set_pose_target(neutral_pose)
    # group.execute(group.plan()[1])
    pass

def plan_path(group, start_pose, scale):
    wpose = copy.deepcopy(start_pose)
    # Determine which two axes to move randomly
    axes = ['x', 'y', 'z']
    factor = 1
    selected_axes = random.sample(axes, 2)  # Randomly pick 2 axes
    if(scale % 2 == 0): factor = 1 
    else: factor = -1
    for axis in selected_axes:
        if (axis == 'z'):
            wpose.position.z += factor *(scale * 0.02)  # First move up (z)
        elif (axis == 'y'):
            wpose.position.y -= factor *(scale * 0.02)  # and sideways (y)
        elif (axis == 'x'):
            wpose.position.x -= factor *(scale * 0.02) # Second move forward/backwards in (x)
    group.set_pose_target(wpose)
    output = group.plan()
    return output[1] , wpose 

def collect_trajectories():
    rospy.init_node('collect_trajectories', anonymous=True)  #Start ROS node for this script.
    #moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.roscpp_initialize(['/home/potato/catkin_ws/src/hw_pkg/src/collect_trajectories_script.py', '__ns:=/my_gen3/'])
    #Initialize RobotCommander and MoveGroupCommander using the namespace and arm group
    #Create interfaces to the robot and planning scene
    robot = moveit_commander.RobotCommander("robot_description")
    scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    group = moveit_commander.MoveGroupCommander("arm", ns=rospy.get_namespace()) # Initialize MoveGroupCommander with the arm group name
    trajectories=[] #initialize list to store planned trajectories
    reach_neutral_pose(group)
    start_pose = group.get_current_pose().pose  #Start pose is the pose the robot starts at.
    if start_pose is None:
        rospy.logerr("Failed to retrieve current pose for trajectory definition")
        return
    #This for loop is completely and utterly correct, output is correct, thank god.
    for i in range(5): #loop to define 5 start and end poses
        print("iteration: " + str(i))
        # Convert trajectory to a serializable format (e.g., list of waypoints)
        #trajectory_plan , end_pose = plan_cartesian_path(group, start_pose, i+1)
        trajectory_message, end_pose = plan_path(group, start_pose, i+1)
        start_pose = end_pose
        #group.execute(trajectory_plan,wait=True)
        trajectories.append(trajectory_message)
    with open('/home/potato/catkin_ws/src/hw_pkg/trajectories.pkl','wb') as file: #open file for writing
        file.truncate(0) #Make sure to empty the file before writing to it.
        pickle.dump(trajectories, file) #save all planned trajectories to file

if __name__=='__main__':
    collect_trajectories()  #Call the defined function.
