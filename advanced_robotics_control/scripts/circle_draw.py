#!/usr/bin/env python2
import rospy
import math
from geometry_msgs.msg import Pose
import time

def draw_line():
    # Initialize ROS node
    rospy.init_node('line_trajectory_publisher', anonymous=True)
    
    # Create a publisher for the topic
    pub = rospy.Publisher('/impedance_control_controller/desired_pose', Pose, queue_size=10)
    
    # Set the rate (Hz) for publishing
    rate = rospy.Rate(20)  # 20 Hz
    
    # Line parameters
    start_x = 0.009  # Fixed x position
    start_y = 0.1846  # Start position in y
    start_z = 0.16    # Fixed z position
    line_length = 0.1  # Length of the line in y direction (adjust as needed)
    duration = 3      # Total duration to complete the line (seconds)
    
    # Calculate how many points we need to publish
    total_points = duration * 20  # 20 Hz for 20 seconds = 400 points
    
    start_time = time.time()
    
    count = 0
    while not rospy.is_shutdown() and (time.time() - start_time) < duration:
        # Calculate the position for this point on the line
        # Linear interpolation from start_y to start_y + line_length
        progress_ratio = float(count) / total_points
        
        # Create the pose message
        pose = Pose()
        
        # Set position (fixed X and Z, only Y changes)
        pose.position.x = start_x
        pose.position.y = start_y 
        pose.position.z = start_z + progress_ratio * line_length
        
        # Set orientation (using the same orientation as in your example)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0  # Valid quaternion representing no rotation
        
        # Publish the message
        pub.publish(pose)
        
        # Increment counter and sleep
        count += 1
        rate.sleep()
        
        # Print progress information
        progress = int((time.time() - start_time) / duration * 100)
        print(progress)
    
    print("\nLinear trajectory completed!")

if __name__ == '__main__':
    try:
        draw_line()
    except rospy.ROSInterruptException:
        pass