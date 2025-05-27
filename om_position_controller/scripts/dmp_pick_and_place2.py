#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import pickle
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from movement_primitives.dmp import CartesianDMP
from movement_primitives.kinematics import Kinematics
import pytransform3d.trajectories as ptr
import tf
import os
import matplotlib.pyplot as plt # Add this import
from mpl_toolkits.mplot3d import Axes3D # Add this import for 3D plotting

# Suppress TF warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import logging
logging.getLogger('tf2_ros').setLevel(logging.ERROR)

class SimpleDMPGenerator:
    def __init__(self, urdf_path, mesh_path=None):
        """Simple DMP Motion Generator with Gripper Control"""
        print("Initializing SimpleDMPGenerator...")
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        
        # Load kinematics
        with open(urdf_path, 'r') as f:
            self.kin = Kinematics(f.read(), mesh_path=mesh_path)
        
        # Joint names for 6DOF arm
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["gripper", "gripper_sub"]  # Both gripper joints for Gazebo
        self.chain = self.kin.create_chain(self.joint_names, "world", "end_effector_link")
        
        # DMP and trajectory storage
        self.dmp = None
        self.gripper_trajectory = None
        
        # ROS setup
        if not rospy.core.is_initialized():
            rospy.init_node('simple_dmp_generator', anonymous=True)
        
        # Publishers for arm and gripper
        self.arm_pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', 
                                     JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher('/open_manipulator_6dof/gripper_controller/command', 
                                         JointTrajectory, queue_size=10)
        
        # Option: Combined publisher if your robot accepts all joints together
        self.combined_pub = rospy.Publisher('/open_manipulator_6dof/joint_trajectory_controller/command', 
                                          JointTrajectory, queue_size=10)
        print("SimpleDMPGenerator initialized successfully!")
    
    def learn_from_bag(self, bag_path, joint_topic, n_weights=10):
        """Learn DMP from rosbag recording"""
        print(f"Learning from bag: {bag_path}")
        
        # Extract data from bag
        transforms, joint_traj, gripper_traj, timestamps = self._extract_bag_data(bag_path, joint_topic)
        
        # Store gripper trajectory
        self.gripper_trajectory = gripper_traj
        
        # Convert transforms to position-quaternion-scale format
        Y = ptr.pqs_from_transforms(transforms)
        
        # Calculate time step
        dt = np.mean(np.diff(timestamps))
        execution_time = timestamps[-1] - timestamps[0]
        
        # Create and train DMP
        self.dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=n_weights)
        self.dmp.imitate(timestamps, Y)
        
        print(f"DMP learned successfully! Execution time: {execution_time:.2f}s")
        print(f"Gripper trajectory has {len(gripper_traj)} points")
        return Y, transforms, joint_traj
    
    def _extract_bag_data(self, bag_path, joint_topic):
        """Extract trajectory data from rosbag"""
        transforms = []
        joint_trajectory = []
        gripper_trajectory = []
        timestamps = []
        
        bag = rosbag.Bag(bag_path)
        for topic, msg, t in bag.read_messages(topics=[joint_topic]):
            # Get arm joints (first 6) and gripper (7th)
            joint_pos = msg.position[:6]
            gripper_pos = msg.position[6] if len(msg.position) > 6 else 0.0
            
            joint_trajectory.append(joint_pos)
            gripper_trajectory.append(gripper_pos)
            transforms.append(self.chain.forward(joint_pos))
            timestamps.append(msg.header.stamp.to_sec())
        
        bag.close()
        
        # Convert to numpy arrays and normalize time
        transforms = np.array(transforms)
        joint_trajectory = np.array(joint_trajectory)
        gripper_trajectory = np.array(gripper_trajectory)
        timestamps = np.array(timestamps)
        timestamps = timestamps - timestamps[0]  # Start from 0
        
        return transforms, joint_trajectory, gripper_trajectory, timestamps
    
    def generate_new_trajectory(self, start_position=None, goal_position=None):
        """Generate new trajectory with optional direct start and goal positions"""
        if self.dmp is None:
            raise ValueError("No DMP learned yet! Call learn_from_bag() first.")
        
        # Get original start and goal as defaults
        new_start = self.dmp.start_y.copy()
        new_goal = self.dmp.goal_y.copy()
        
        # Set new positions if provided
        if start_position is not None:
            new_start[:3] = np.array(start_position)
            print(f"Start position set to: {start_position}")
            
        if goal_position is not None:
            new_goal[:3] = np.array(goal_position)
            print(f"Goal position set to: {goal_position}")
        
        # Generate trajectory
        self.dmp.start_y = new_start
        self.dmp.goal_y = new_goal
        
        T, Y = self.dmp.open_loop()
        trajectory = ptr.transforms_from_pqs(Y)
        
        print(f"Generated trajectory with {len(trajectory)} points")
        return T, trajectory
    
    def trajectory_to_joints(self, trajectory, q0=None):
        """Convert Cartesian trajectory to joint space using IK"""
        if q0 is None:
            q0 = np.array([0.0, -0.78, 1.5, 0.0, 0.8, 0.0])
        
        print("Solving inverse kinematics...")
        
        # Use random state for consistency
        random_state = np.random.RandomState(0)
        joint_trajectory = self.chain.inverse_trajectory(
            trajectory, random_state=random_state, orientation_weight=1.0
        )
        
        print(f"IK solved for {len(joint_trajectory)} points")
        return joint_trajectory
    
    def interpolate_gripper_trajectory(self, target_length):
        """Interpolate gripper trajectory to match joint trajectory length"""
        if self.gripper_trajectory is None or len(self.gripper_trajectory) == 0:
            print("No gripper trajectory available, using default closed position (0.0)")
            return np.zeros(target_length)
        
        # If lengths match, return as is
        if len(self.gripper_trajectory) == target_length:
            return self.gripper_trajectory
        
        # Interpolate to match target length
        original_indices = np.linspace(0, len(self.gripper_trajectory) - 1, len(self.gripper_trajectory))
        target_indices = np.linspace(0, len(self.gripper_trajectory) - 1, target_length)
        interpolated = np.interp(target_indices, original_indices, self.gripper_trajectory)
        
        print(f"Interpolated gripper trajectory from {len(self.gripper_trajectory)} to {target_length} points")
        return interpolated
    
    def publish_trajectory_combined(self, joint_trajectory, duration_per_point=0.1):
        """Publish arm and gripper trajectory together (if your robot supports this)"""
        if rospy.is_shutdown():
            print("ROS is shutdown, cannot publish")
            return
        
        # Get gripper trajectory matching joint trajectory length
        gripper_traj = self.interpolate_gripper_trajectory(len(joint_trajectory))
        
        # Create trajectory message with all joints
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = self.joint_names + self.gripper_joint_names
        
        # Add trajectory points
        for i, joint_pos in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            # Combine arm joints with gripper position (same value for both gripper joints)
            gripper_value = gripper_traj[i]
            point.positions = joint_pos.tolist() + [gripper_value, gripper_value]
            point.time_from_start = rospy.Duration((i + 1) * duration_per_point)
            traj_msg.points.append(point)
        
        # Publish
        self.combined_pub.publish(traj_msg)
        print(f"Published combined trajectory with {len(joint_trajectory)} points (arm + gripper)")
    
    def publish_trajectory_separate(self, joint_trajectory, duration_per_point=0.1):
        """Publish arm and gripper trajectories separately"""
        if rospy.is_shutdown():
            print("ROS is shutdown, cannot publish")
            return
        
        # Get gripper trajectory matching joint trajectory length
        gripper_traj = self.interpolate_gripper_trajectory(len(joint_trajectory))
        
        # Create arm trajectory message
        arm_traj_msg = JointTrajectory()
        arm_traj_msg.header.stamp = rospy.Time.now()
        arm_traj_msg.joint_names = self.joint_names
        
        # Create gripper trajectory message
        gripper_traj_msg = JointTrajectory()
        gripper_traj_msg.header.stamp = rospy.Time.now()
        gripper_traj_msg.joint_names = self.gripper_joint_names
        
        # Add trajectory points
        for i, joint_pos in enumerate(joint_trajectory):
            # Arm point
            arm_point = JointTrajectoryPoint()
            arm_point.positions = joint_pos.tolist()
            arm_point.time_from_start = rospy.Duration((i + 1) * duration_per_point)
            arm_traj_msg.points.append(arm_point)
            
            # Gripper point
            gripper_point = JointTrajectoryPoint()
            gripper_value = gripper_traj[i]
            gripper_point.positions = [gripper_value, gripper_value]  # Same value for both gripper joints
            gripper_point.time_from_start = rospy.Duration((i + 1) * duration_per_point)
            gripper_traj_msg.points.append(gripper_point)
        
        # Publish both trajectories
        self.arm_pub.publish(arm_traj_msg)
        self.gripper_pub.publish(gripper_traj_msg)
        print(f"Published separate trajectories: arm ({len(joint_trajectory)} points) and gripper ({len(gripper_traj)} points)")
    
    def publish_trajectory_step_by_step(self, joint_trajectory, duration_per_point=0.1, publish_method='separate'):
        """Publish trajectory step by step for debugging"""
        if rospy.is_shutdown():
            print("ROS is shutdown, cannot publish")
            return
        
        # Get gripper trajectory matching joint trajectory length
        gripper_traj = self.interpolate_gripper_trajectory(len(joint_trajectory))
        
        rate = rospy.Rate(1.0 / duration_per_point)
        
        for i, joint_pos in enumerate(joint_trajectory):
            if rospy.is_shutdown():
                break
            
            if publish_method == 'combined':
                # Create combined single point trajectory
                traj_msg = JointTrajectory()
                traj_msg.header.stamp = rospy.Time.now()
                traj_msg.joint_names = self.joint_names + self.gripper_joint_names
                
                point = JointTrajectoryPoint()
                gripper_value = gripper_traj[i]
                point.positions = joint_pos.tolist() + [gripper_value, gripper_value]
                point.time_from_start = rospy.Duration(duration_per_point)
                traj_msg.points.append(point)
                
                self.combined_pub.publish(traj_msg)
                
            else:  # separate
                # Create arm trajectory
                arm_traj = JointTrajectory()
                arm_traj.header.stamp = rospy.Time.now()
                arm_traj.joint_names = self.joint_names
                
                arm_point = JointTrajectoryPoint()
                arm_point.positions = joint_pos.tolist()
                arm_point.time_from_start = rospy.Duration(duration_per_point)
                arm_traj.points.append(arm_point)
                
                # Create gripper trajectory
                gripper_traj_msg = JointTrajectory()
                gripper_traj_msg.header.stamp = rospy.Time.now()
                gripper_traj_msg.joint_names = self.gripper_joint_names
                
                gripper_point = JointTrajectoryPoint()
                gripper_value = gripper_traj[i]
                gripper_point.positions = [gripper_value, gripper_value]  # Same value for both gripper joints
                gripper_point.time_from_start = rospy.Duration(duration_per_point)
                gripper_traj_msg.points.append(gripper_point)
                
                # Publish both
                self.arm_pub.publish(arm_traj)
                self.gripper_pub.publish(gripper_traj_msg)
            
            print(f"Step {i+1}/{len(joint_trajectory)} - Arm: {joint_pos[:3]}, Gripper: {gripper_traj[i]:.3f}")
            rate.sleep()
    
    def publish_trajectory(self, joint_trajectory, duration_per_point=0.1, method='separate', step_by_step=False):
        """
        Main trajectory publishing method with multiple options
        
        Args:
            joint_trajectory: Array of joint positions
            duration_per_point: Time between trajectory points
            method: 'separate' or 'combined' - how to publish arm and gripper
            step_by_step: If True, publish point by point for debugging
        """
        if step_by_step:
            self.publish_trajectory_step_by_step(joint_trajectory, duration_per_point, method)
        elif method == 'combined':
            self.publish_trajectory_combined(joint_trajectory, duration_per_point)
        else:
            self.publish_trajectory_separate(joint_trajectory, duration_per_point)
    
    def save_dmp(self, filepath):
        """Save learned DMP to file"""
        if self.dmp is None:
            print("No DMP to save!")
            return
            
        data = {
            'dmp': self.dmp,
            'gripper_trajectory': self.gripper_trajectory
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)
        print(f"DMP saved to {filepath}")
    
    def load_dmp(self, filepath):
        """Load DMP from file"""
        try:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
            
            if isinstance(data, dict):
                self.dmp = data.get('dmp')
                self.gripper_trajectory = data.get('gripper_trajectory')
            else:
                self.dmp = data  # Old format
                self.gripper_trajectory = None
                
            print(f"DMP loaded from {filepath}")
        except Exception as e:
            print(f"Failed to load DMP: {e}")

def plot_trajectories(original_xyz, new_goal_xyz, ik_calculated_xyz, original_dmp_learned_goal_marker, new_target_goal_marker):
    """Plots three trajectories: original DMP, new goal DMP, and IK-derived."""
    fig = plt.figure(figsize=(20, 7))
    plt.suptitle("DMP Trajectory Analysis", fontsize=16)

    # Plot 1: Original DMP Trajectory
    ax1 = fig.add_subplot(131, projection='3d')
    if original_xyz.size > 0:
        ax1.plot(original_xyz[:, 0], original_xyz[:, 1], original_xyz[:, 2], label='Original DMP Trajectory', color='blue')
        ax1.scatter(original_xyz[0, 0], original_xyz[0, 1], original_xyz[0, 2], color='green', s=100, label='Start', zorder=5)
        ax1.scatter(original_xyz[-1, 0], original_xyz[-1, 1], original_xyz[-1, 2], color='red', s=100, label='End of Traj (Actual)', zorder=5)
    if original_dmp_learned_goal_marker is not None:
        ax1.scatter(original_dmp_learned_goal_marker[0], original_dmp_learned_goal_marker[1], original_dmp_learned_goal_marker[2],
                    color='darkblue', marker='x', s=150, label='DMP Learned Goal', zorder=5)
    ax1.set_title('1. Original DMP Trajectory')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.legend()

    # Plot 2: DMP Trajectory to New Goal
    ax2 = fig.add_subplot(132, projection='3d')
    if new_goal_xyz.size > 0:
        ax2.plot(new_goal_xyz[:, 0], new_goal_xyz[:, 1], new_goal_xyz[:, 2], label='DMP Traj. to New Goal', color='orange')
        ax2.scatter(new_goal_xyz[0, 0], new_goal_xyz[0, 1], new_goal_xyz[0, 2], color='green', s=100, label='Start', zorder=5)
        ax2.scatter(new_goal_xyz[-1, 0], new_goal_xyz[-1, 1], new_goal_xyz[-1, 2], color='red', s=100, label='End of Traj (Actual)', zorder=5)
    if new_target_goal_marker is not None:
        ax2.scatter(new_target_goal_marker[0], new_target_goal_marker[1], new_target_goal_marker[2],
                    color='darkorange', marker='x', s=150, label='Target New Goal', zorder=5)
    ax2.set_title('2. DMP Trajectory to New Goal')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.legend()

    # Plot 3: IK-Derived Trajectory (Forward Kinematics)
    ax3 = fig.add_subplot(133, projection='3d')
    if ik_calculated_xyz.size > 0:
        ax3.plot(ik_calculated_xyz[:, 0], ik_calculated_xyz[:, 1], ik_calculated_xyz[:, 2], label='IK-Derived Traj. (FK)', color='purple')
        ax3.scatter(ik_calculated_xyz[0, 0], ik_calculated_xyz[0, 1], ik_calculated_xyz[0, 2], color='green', s=100, label='Start', zorder=5)
        ax3.scatter(ik_calculated_xyz[-1, 0], ik_calculated_xyz[-1, 1], ik_calculated_xyz[-1, 2], color='red', s=100, label='End of Traj (Actual)', zorder=5)
    if new_target_goal_marker is not None: # Show the same target goal for reference
        ax3.scatter(new_target_goal_marker[0], new_target_goal_marker[1], new_target_goal_marker[2],
                    color='darkorange', marker='x', s=150, label='Target New Goal', zorder=5)
    ax3.set_title('3. IK-Derived Trajectory (via FK)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_zlabel('Z (m)')
    ax3.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust layout to make space for suptitle
    plt.show()

def get_xyz_from_world_to_three():
    print("Getting blue cube position...")
    if not rospy.core.is_initialized():
        rospy.init_node('tf_xyz_fetcher', anonymous=True)

    listener = tf.TransformListener()
    
    try:
        # Wait until the transform becomes available
        print("Waiting for transform /world -> /blue_cube...")
        listener.waitForTransform('/world', '/blue_cube', rospy.Time(0), rospy.Duration(10.0))
        
        # Get the latest available transform
        trans, _ = listener.lookupTransform('/world', '/blue_cube', rospy.Time(0))
        print(f"Blue cube position: {trans}")
        return trans
    except Exception as e:
        print(f"Failed to get blue cube position: {e}")
        return [0.2, -0.1, 0.015]  # Default position

def main():
    """Main execution function"""
    print("="*50)
    print("STARTING DMP PICK AND PLACE")
    print("="*50)
    
    # Configuration
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    bag_path = '/root/catkin_ws/src/recordings/pick11.bag'
    dmp_save_path = '/root/catkin_ws/src/recordings/simple_dmp.pkl'
    
    try:
        # Initialize DMP generator
        print("\n=== STEP 1: Learning from demonstration ===")
        dmp_gen = SimpleDMPGenerator(urdf_path, mesh_path)
        
        Y, transforms, joint_traj = dmp_gen.learn_from_bag(
            bag_path, 
            '/gravity_compensation_controller/traj_joint_states'
        )
        
        # Save the learned DMP
        dmp_gen.save_dmp(dmp_save_path)

        # Get XYZ of the original DMP trajectory (from PQS format)
        original_dmp_xyz = Y[:, :3]
        original_dmp_learned_goal_marker = dmp_gen.dmp.goal_y[:3].copy() #Learned goal from DMP

        # Step 2: Generate new trajectory with modifications
        print("\n=== STEP 2: Generating new trajectory ===")
        
        # Get the blue cube position
        trans_three = get_xyz_from_world_to_three()

        # Get original goal for reference
        original_goal = dmp_gen.dmp.goal_y.copy()
        print(f"Original goal: {original_goal}")
        
        # Calculate the goal offset to reach the blue cube
        new_goal = np.array(trans_three) 
   
        
        print(f"Goal onewffset: {new_goal}")
        
        # Generate new trajectory using the existing method
        T, trajectory_to_new_goal = dmp_gen.generate_new_trajectory(
            None,  # No start offset
            new_goal
        )
        
        # Step 3: Convert to joint space
        print("\\n=== STEP 3: Converting to joint space ===")
        joint_trajectory_for_new_goal = dmp_gen.trajectory_to_joints(trajectory_to_new_goal)

        # Get XYZ of the DMP trajectory to the new goal
        # trajectory_to_new_goal is a series of transformation matrices
        pqs_for_new_goal_traj = ptr.pqs_from_transforms(trajectory_to_new_goal)
        dmp_to_new_goal_xyz = pqs_for_new_goal_traj[:, :3]

        # Step 3b: Perform Forward Kinematics on the IK solution to get achievable Cartesian trajectory
        print("\\n=== STEP 3b: Calculating FK from IK solution ===")
        ik_calculated_cartesian_points = []
        if joint_trajectory_for_new_goal is not None and len(joint_trajectory_for_new_goal) > 0:
            for joint_pos in joint_trajectory_for_new_goal:
                transform_matrix = dmp_gen.chain.forward(joint_pos)
                ik_calculated_cartesian_points.append(transform_matrix[:3, 3]) # Extract X, Y, Z
            ik_calculated_xyz = np.array(ik_calculated_cartesian_points)
        else:
            print("IK solution is empty, cannot calculate FK-based trajectory.")
            ik_calculated_xyz = np.array([])


        # Step 4: Plotting trajectories
        print("\\n=== STEP 4: Plotting Trajectories ===")
        plot_trajectories(original_dmp_xyz, dmp_to_new_goal_xyz, ik_calculated_xyz,
                          original_dmp_learned_goal_marker, new_goal)


        # Step 5: Publish to robot
        print("\\n=== STEP 5: Publishing to robot ===")
        print("Publishing in 3 seconds...")
        rospy.sleep(3)

        dmp_gen.publish_trajectory(joint_trajectory_for_new_goal, duration_per_point=0.1,
                                 method='separate', step_by_step=False)

        print("\\n" + "="*50)
        print("EXECUTION COMPLETED SUCCESSFULLY!")
        print("="*50)
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")

if __name__ == '__main__':
    main()