#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Define parameters
num_timesteps = 500
total_time = 10.0
dt = total_time / num_timesteps
t = np.linspace(0, total_time, num_timesteps)

# Initialize trajectory with zeros
desired_positions = np.zeros((num_timesteps, 6))

# Sine wave properties
amplitude = 1.0
period = 10.0
omega = 2 * np.pi / period

# Assign sine waves to the last three joints with different phase shifts
desired_positions[:, 3] = amplitude * np.sin(omega * t + 0)
desired_positions[:, 4] = amplitude * np.sin(omega * t + np.pi / 3)
desired_positions[:, 5] = amplitude * np.sin(omega * t + 2 * np.pi / 3)

# Correct joint names (no underscores)
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

def publish_joint_trajectory():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    
    # Publish to the arm controller's command topic
    pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', JointTrajectory, queue_size=10)
    
    rospy.sleep(1.0)  # Wait for publisher to be ready
    
    rospy.loginfo("Publishing joint trajectory...")
    
    # Create trajectory message
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names
    
    # Add all trajectory points
    for t_idx in range(num_timesteps):
        point = JointTrajectoryPoint()
        point.positions = desired_positions[t_idx, :].tolist()
        point.time_from_start = rospy.Duration(t_idx * dt)
        traj_msg.points.append(point)
    
    # Publish the complete trajectory
    pub.publish(traj_msg)
    rospy.loginfo("Published trajectory with %d points", len(traj_msg.points))

if __name__ == '__main__':
    try:
        publish_joint_trajectory()
        rospy.sleep(2.0)  # Keep node alive briefly
    except rospy.ROSInterruptException:
        pass