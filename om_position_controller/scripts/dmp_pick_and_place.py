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

class SimpleDMPGenerator:
    def __init__(self, urdf_path, mesh_path=None):
        """Simple DMP Motion Generator"""
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        
        # Load kinematics
        with open(urdf_path, 'r') as f:
            self.kin = Kinematics(f.read(), mesh_path=mesh_path)
        
        # Joint names for 6DOF arm
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.chain = self.kin.create_chain(self.joint_names, "world", "end_effector_link")
        
        # DMP and trajectory storage
        self.dmp = None
        self.gripper_trajectory = None
        
        # ROS setup
        rospy.init_node('simple_dmp_generator', anonymous=True)
        self.pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', 
                                 JointTrajectory, queue_size=10)
        
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
    
    def generate_new_trajectory(self, start_offset=None, goal_offset=None):
        """Generate new trajectory with optional position offsets"""
        if self.dmp is None:
            raise ValueError("No DMP learned yet! Call learn_from_bag() first.")
        
        # Get original start and goal
        new_start = self.dmp.start_y.copy()
        new_goal = self.dmp.goal_y.copy()
        
        # Apply offsets if provided
        if start_offset is not None:
            new_start[:3] += np.array(start_offset)
            print(f"Start offset applied: {start_offset}")
            
        if goal_offset is not None:
            new_goal[:3] += np.array(goal_offset)
            print(f"Goal offset applied: {goal_offset}")
        
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
    
    def publish_trajectory(self, joint_trajectory, duration_per_point=0.1):
        """Publish joint trajectory to robot controller"""
        if rospy.is_shutdown():
            print("ROS is shutdown, cannot publish")
            return
            
        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = self.joint_names
        
        # Add trajectory points
        for i, joint_pos in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joint_pos.tolist()
            point.time_from_start = rospy.Duration((i + 1) * duration_per_point)
            traj_msg.points.append(point)
        
        # Publish
        self.pub.publish(traj_msg)
        print(f"Published trajectory with {len(joint_trajectory)} points")
        
        # Optional: publish individual points for debugging
        rate = rospy.Rate(1.0 / duration_per_point)
        for i, joint_pos in enumerate(joint_trajectory):
            if rospy.is_shutdown():
                break
                
            # Create single point trajectory
            single_traj = JointTrajectory()
            single_traj.header.stamp = rospy.Time.now()
            single_traj.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = joint_pos.tolist()
            point.time_from_start = rospy.Duration(duration_per_point)
            single_traj.points.append(point)
            
            self.pub.publish(single_traj)
            print(f"Step {i+1}/{len(joint_trajectory)}")
            rate.sleep()
    
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

def main():
    """Main execution function"""
    # Configuration
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    bag_path = '/root/catkin_ws/src/recordings/pick11.bag'
    dmp_save_path = '/root/catkin_ws/src/recordings/simple_dmp.pkl'
    
    try:
        # Initialize DMP generator
        dmp_gen = SimpleDMPGenerator(urdf_path, mesh_path)
        
        # Step 1: Learn from demonstration
        print("=== STEP 1: Learning from demonstration ===")
        Y, transforms, joint_traj = dmp_gen.learn_from_bag(
            bag_path, 
            '/gravity_compensation_controller/traj_joint_states'
        )
        
        # Save the learned DMP
        dmp_gen.save_dmp(dmp_save_path)
        
        # Step 2: Generate new trajectory with modifications
        print("\n=== STEP 2: Generating new trajectory ===")
        # Example: move start up by 3cm, goal up by 3cm
        T, new_trajectory = dmp_gen.generate_new_trajectory(
            start_offset=[0.01, 0.0, 0.03],
            goal_offset=[0.0, 0.0, 0.03]
        )
        
        # Step 3: Convert to joint space
        print("\n=== STEP 3: Converting to joint space ===")
        joint_trajectory = dmp_gen.trajectory_to_joints(new_trajectory)
        
        # Step 4: Publish to robot
        print("\n=== STEP 4: Publishing to robot ===")
        print("Publishing in 3 seconds...")
        rospy.sleep(3)
        
        dmp_gen.publish_trajectory(joint_trajectory, duration_per_point=0.1)
        
        print("\n=== DONE! ===")
        
    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")

if __name__ == '__main__':
    main()