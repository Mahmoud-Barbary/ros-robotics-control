# ROS Robotics Control

This repository contains a set of ROS (Robot Operating System) projects developed as part of a robotics coursework assignment. It demonstrates:

- Real-time obstacle avoidance on a TurtleBot3 using LiDAR in Gazebo
- Trajectory planning and execution on a Kinova Gen3 manipulator using MoveIt!

Each script is modular, simulation-ready, and useful for both education and demonstration.

---

## 🐢 TurtleBot3 Obstacle Avoidance

This ROS script controls a TurtleBot3 in Gazebo using LiDAR sensor data from `/scan` to avoid obstacles.
A ROS node subscribes to `/scan` (LiDAR) and publishes to `/cmd_vel` to move the robot forward until an obstacle is detected, then turns to avoid it.

📁 Folder: `turtlebot_obstacle_avoidance/`  
▶️ Launch: 
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch your_package avoid_obstacles.launch
```

### 🚀 How it works
- If no obstacle is detected, robot moves forward
- If obstacle < 0.7m, it turns right to avoid
- Continues until obstacle-free path is found
- ✅ Subscribes to `/scan`
- ✅ Publishes to `/cmd_vel`
- ✅ Avoids obstacles by turning when within threshold distance

---

## 🤖 Kinova Trajectory Planning & Execution

Two scripts using `moveit_commander` for planning and executing 5 random Cartesian trajectories on a Kinova Gen3 robotic arm in simulation.

📁 Folder: `kinova_trajectory_planning/`  
Scripts:
- `collect_trajectories_script.py`: Plans and saves 5 paths
- `load_trajectories_script.py`: Loads and executes them sequentially
### ▶️ Run in Gazebo
```bash
roslaunch kortex_gazebo spawn_kortex_robot.launch gripper:=robotiq_2f_85
python3 collect_trajectories_script.py
python3 load_trajectories_script.py
```

Uses `pickle` for serialization of trajectory messages.

### 🧠 Concepts Used
- MoveIt! trajectory planning
- Cartesian pose manipulation
- Synchronizing joints to match trajectory start
- RobotCommander + PlanningSceneInterface

---

## 🛠 Requirements

- ROS Noetic (Ubuntu 20.04)
- Gazebo with TurtleBot3 and Kinova simulation
- `ros_kortex` package for Kinova control
- Python packages: `rospy`, `moveit_commander`, `pickle`

See ROS, Gazebo and TurtleBot3 documentations for set-up.
---
