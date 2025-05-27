#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimpleHomeMover:
    def __init__(self):
        """Simple robot home position mover"""
        print("Initializing SimpleHomeMover...")
        
        # Joint names for 6DOF arm
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["gripper", "gripper_sub"]
        
        # Home position joint angles
        self.home_position = np.array([0.0, -0.78, 1.5, 0.0, 0.8, 0.0])
        self.home_position = np.array([-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194])  # Ensure angles are within limits
        self.gripper_open_position = 0.01  # Open gripper position
        
        print("Setting up ROS node...")
        
        # ROS setup
        try:
            rospy.init_node('simple_home_mover', anonymous=True)
            print("ROS node initialized successfully")
        except rospy.exceptions.ROSException as e:
            print(f"Failed to initialize ROS node: {e}")
            raise
        
        print("Creating publishers...")
        
        # Publishers for arm and gripper
        self.arm_pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', 
                                     JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher('/open_manipulator_6dof/gripper_controller/command', 
                                         JointTrajectory, queue_size=10)
        
        print("Publishers created, waiting for connections...")
        
        # Wait for publishers to connect
        rospy.sleep(2.0)
        
        # Check if publishers have subscribers
        arm_connections = self.arm_pub.get_num_connections()
        gripper_connections = self.gripper_pub.get_num_connections()
        
        print(f"Arm publisher connections: {arm_connections}")
        print(f"Gripper publisher connections: {gripper_connections}")
        
        if arm_connections == 0:
            print("WARNING: No subscribers for arm controller!")
            print("Make sure the robot controllers are running.")
        
        if gripper_connections == 0:
            print("WARNING: No subscribers for gripper controller!")
            print("Make sure the gripper controller is running.")
        
        print("SimpleHomeMover initialized successfully!")
    
    def move_to_home(self, duration=5.0):
        """Move robot to home position with gripper open"""
        print("Moving to home position...")
        
        # Create arm trajectory message
        arm_traj_msg = JointTrajectory()
        arm_traj_msg.header.stamp = rospy.Time.now()
        arm_traj_msg.joint_names = self.joint_names
        
        # Create single trajectory point for home position
        arm_point = JointTrajectoryPoint()
        arm_point.positions = self.home_position.tolist()
        arm_point.time_from_start = rospy.Duration(duration)
        arm_traj_msg.points.append(arm_point)
        
        # Create gripper trajectory message
        gripper_traj_msg = JointTrajectory()
        gripper_traj_msg.header.stamp = rospy.Time.now()
        gripper_traj_msg.joint_names = self.gripper_joint_names
        
        # Create single trajectory point for open gripper
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [self.gripper_open_position, self.gripper_open_position]
        gripper_point.time_from_start = rospy.Duration(duration)
        gripper_traj_msg.points.append(gripper_point)
        
        # Publish both trajectories
        self.arm_pub.publish(arm_traj_msg)
        self.gripper_pub.publish(gripper_traj_msg)
        
        print(f"Home position command published!")
        print(f"Joint angles: {self.home_position}")
        print(f"Gripper position: {self.gripper_open_position} (open)")
        print(f"Duration: {duration} seconds")
    
    def move_step_by_step(self, intermediate_steps=5, step_duration=1.0):
        """Move to home position with intermediate steps for smoother motion"""
        print("Moving to home position with intermediate steps...")
        
        # You can implement this if you want smoother motion
        # For now, we'll just use the simple approach
        self.move_to_home(duration=step_duration * intermediate_steps)

def main():
    """Main execution function"""
    print("Starting home mover script...")
    
    try:

        
        print("ROS master detected, initializing...")
        
        # Initialize home mover
        home_mover = SimpleHomeMover()
        
        print("=== MOVING TO HOME POSITION ===")
        print("Robot will move to home position in 3 seconds...")
        
        # Countdown
        for i in range(3, 0, -1):
            print(f"Moving in {i}...")
            rospy.sleep(1)
        
        # Move to home position
        home_mover.move_to_home(duration=5.0)
        
        print("Home position command sent!")
        print("The robot should now move to home position with gripper open.")
        print("Waiting for movement to complete...")
        
        # Keep node alive and show status
        for i in range(6):
            print(f"Progress: {i+1}/6 seconds...")
            rospy.sleep(1)
        
        print("Movement completed!")
        print("Script finished successfully!")
        
    except rospy.ROSException as e:
        print(f"ROS Error: {e}")
        print("Make sure ROS master is running: roscore")
    except Exception as e:
        print(f"Unexpected Error: {e}")
        import traceback
        traceback.print_exc()
    except KeyboardInterrupt:
        print("Interrupted by user")

if __name__ == '__main__':
    main()