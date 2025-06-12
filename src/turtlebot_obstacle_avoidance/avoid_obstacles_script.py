#!/usr/bin/env python3
#IMPORTANT NOTE: this code runs only after pressing Ctrl+C in terminal to stop teleop control.
# Import ROS libraries
import rospy  #ROS Python library, easier than C++ for me
from sensor_msgs.msg import LaserScan  #Gets the LiDAR sensor data specified for assignment.
from geometry_msgs.msg import Twist  #Sends messages to specify robot's movement, like Robot Control lecture.


def lidar_callback(data): #Callback function that processes LiDAR data and sends movement commands
    
    threshold_distance = 0.7 #Threshold distance for obstacle detection in meters, I like 7.
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Create a publisher to send commands to the robot's velocity controller
    move_cmd = Twist() #Main message for movement commands. Don't understand this as much yet.
    #Takes angle of bot orientation as input, outputs distance to obstacle from that angle.
    #First distance was just data.ranges[0], but that causes problems so we get obstacles at an angle range as well.
    front_distance = min(data.ranges[0], data.ranges[20], data.ranges[340]) #Distance to the obstacle directly in front of robot.
    if front_distance < threshold_distance:#Obstacle is detected within the threshold distance
        move_cmd.linear.x = 0.0 # Stop moving forward
        move_cmd.angular.z = -0.5 #Turn the robot to the right (counter-clockwise)
    else:
        move_cmd.linear.x = 0.5 #No obstacle, move.
        move_cmd.angular.z = 0.0 #No turning.
        front_distance = min(data.ranges[0], data.ranges[10], data.ranges[350])
    cmd_pub.publish(move_cmd)#Publish (send) movement command with the specified velocities.

if __name__ == '__main__': # Main entry point
    try:
        rospy.init_node('avoid_obstacles_script', anonymous=True) #Initialize our ROS node.
        # Create a subscriber to listen to the LiDAR data on the '/scan' topic
        #Listen to LiDAR data, call the function when you receive it.
        rospy.Subscriber('/scan', LaserScan, lidar_callback)
        
        
        rospy.spin()  #Keep the node running until stopped (like a spinlock)
    except rospy.ROSInterruptException:
        pass
