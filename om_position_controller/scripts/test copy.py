import numpy as np
import pytransform3d.visualizer as pv
import pytransform3d.trajectories as ptr
from movement_primitives.kinematics import Kinematics
import rosbag
from tf.transformations import quaternion_matrix
from movement_primitives.dmp import CartesianDMP
import pickle
import os
import time
from scipy.interpolate import interp1d
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
import matplotlib.pyplot as plt

#-------------------------------------- Classes --------------------------------------# 

class DMPMotionGenerator:
    def __init__(self, urdf_path, mesh_path=None, joint_names=None, base_link="world", end_effector_link="end_effector_link"):
        """
        Initialize DMP Motion Generator for Gazebo simulation
        
        Parameters:
        -----------
        urdf_path : str
            Path to the URDF file
        mesh_path : str, optional
            Path to mesh files
        joint_names : list, optional
            List of joint names to use
        base_link : str
            Name of the base link
        end_effector_link : str
            Name of the end effector link
        """
        print("Initializing DMPMotionGenerator for Gazebo...")
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        self.kin = self._load_kinematics(urdf_path, mesh_path)
        
        # Joint names for 6DOF arm
        self.joint_names = joint_names or ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["gripper", "gripper_sub"]  # Both gripper joints for Gazebo
        self.base_link = base_link
        self.end_effector_link = end_effector_link
        self.chain = self.kin.create_chain(self.joint_names, base_link, end_effector_link)
        self.dmp = None
        self.IK_joint_trajectory = None
        self.gripper_trajectory = None
        
        # ROS setup
        if not rospy.core.is_initialized():
            rospy.init_node('dmp_motion_generator', anonymous=True)

        
    def _load_kinematics(self, urdf_path, mesh_path=None):
        """Load robot kinematics from URDF"""
        with open(urdf_path, 'r') as f:
            return Kinematics(f.read(), mesh_path=mesh_path)

    def learn_from_rosbag(self, bag_path, joint_topic, dt=None, n_weights=10):
        """Learn DMP from rosbag recording"""
        transforms, joint_trajectory, gripper_trajectory, time_stamp = self._process_rosbag(bag_path, joint_topic)
        self.gripper_trajectory = gripper_trajectory
        
        print(f"Transforms shape: {transforms.shape}")
        # Convert transforms to PQS representation
        Y = ptr.pqs_from_transforms(transforms[10:,:,:])
        if dt is None:
            dt = 1/self.frequency
        # Create and train DMP
        self.dmp = CartesianDMP(execution_time=max(time_stamp), dt=dt, n_weights_per_dim=n_weights)
        self.dmp.imitate(time_stamp[10:], Y)
        
        return Y, transforms, joint_trajectory, gripper_trajectory

    def _process_rosbag(self, bag_path, joint_topic):
        """Process rosbag and extract trajectories"""
        transforms = []
        joint_trajectory = []
        gripper_trajectory = []
        time_stamp = []
        
        print(f"Reading bag file: {bag_path}")
        bag = rosbag.Bag(bag_path)
        for topic, msg, t in bag.read_messages(topics=[joint_topic]):
            joint_pos = msg.position[:6]
            gripper_pos = msg.position[6] if len(msg.position) > 6 else 0.0
            joint_trajectory.append(joint_pos)
            gripper_trajectory.append(gripper_pos)

            transforms.append(self.chain.forward(joint_pos))
            time_stamp.append(msg.header.stamp.to_sec())    
        bag.close()
        
        # Convert to numpy arrays
        transforms = np.array(transforms)
        joint_trajectory = np.array(joint_trajectory)
        gripper_trajectory = np.array(gripper_trajectory)
        time_stamp = np.array(time_stamp)
        
        dt = []
        for i in range(1, time_stamp.shape[0]):
            dt.append(time_stamp[i]- time_stamp[i-1])
        self.frequency = 1/ np.average(np.array(dt))
        
        # First filter outliers
        positions = np.array([T[:3, 3] for T in transforms])
        mask, _ = self.remove_outliers_mad(positions, threshold=5.0)
        
        # Then normalize time (important to do it in this order)
        filtered_time = time_stamp[mask]
        normalized_time = filtered_time - filtered_time[0]
        
        return transforms[mask], joint_trajectory[mask], gripper_trajectory[mask], normalized_time

    def remove_outliers_mad(self, data, threshold=3.5):
        """Remove outliers using Median Absolute Deviation"""
        median = np.median(data, axis=0)
        diff = np.abs(data - median)
        mad = np.median(diff, axis=0)
        modified_z_score = 0.6745 * diff / (mad + 1e-6)
        mask = np.all(modified_z_score < threshold, axis=1)
        return mask, data[mask]

    def generate_trajectory(self, start_y=None, goal_y=None):
        """
        Generate trajectory using the learned DMP
        
        Parameters:
        -----------
        start_y : array-like, shape (7,)
            Start state in PQS format [x,y,z,qw,qx,qy,qz]
        goal_y : array-like, shape (7,)
            Goal state in PQS format [x,y,z,qw,qx,qy,qz]
        """
        print(f"Generating trajectory")
        if self.dmp is None:
            raise ValueError("No DMP model available. Learn or load a model first.")
            
        if start_y is not None:
            self.dmp.start_y = start_y
            print(f"Using custom start: {start_y}")
        else:
            print(f"Using default start: {self.dmp.start_y}")
            
        if goal_y is not None:
            self.dmp.goal_y = goal_y
            print(f"Using custom goal: {goal_y}")
        else:
            print(f"Using default goal: {self.dmp.goal_y}")
        
        T, Y = self.dmp.open_loop()
        trajectory = ptr.transforms_from_pqs(Y)
        return T, trajectory

    def save_dmp(self, filepath):
        """Save the learned DMP and the associated gripper trajectory to file."""
        if self.dmp is None:
            rospy.logerr("No DMP model available to save.")
            return
        if self.gripper_trajectory is None:
            rospy.logwarn("Gripper trajectory not available or not learned. Saving None for gripper_trajectory.")

        data_to_save = {
            'dmp': self.dmp,
            'gripper_trajectory': self.gripper_trajectory
        }
        try:
            with open(filepath, 'wb') as f:
                pickle.dump(data_to_save, f)
            rospy.loginfo(f"DMP and gripper trajectory saved to {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to save DMP data to {filepath}: {e}")

    def load_dmp(self, filepath):
        """Load a DMP and the associated gripper trajectory from file."""
        rospy.loginfo(f"Loading DMP data from {filepath}")
        try:
            with open(filepath, 'rb') as f:
                loaded_data = pickle.load(f)

            # Check if the loaded data is a dictionary (new format) or just the DMP (old format)
            if isinstance(loaded_data, dict):
                if 'dmp' in loaded_data:
                    self.dmp = loaded_data['dmp']
                else:
                    rospy.logerr("Loaded dictionary is missing 'dmp' key.")
                    self.dmp = None

                if 'gripper_trajectory' in loaded_data:
                    self.gripper_trajectory = loaded_data['gripper_trajectory']
                    if self.gripper_trajectory is not None:
                         rospy.loginfo(f"Gripper trajectory loaded ({len(self.gripper_trajectory)} points).")
                    else:
                         rospy.loginfo("Loaded None for gripper trajectory.")
                else:
                    rospy.logwarn("Loaded dictionary is missing 'gripper_trajectory' key. Setting to None.")
                    self.gripper_trajectory = None
            else:
                # Assume old format (only DMP object was saved)
                rospy.logwarn("Loading old DMP format (only DMP object found). Gripper trajectory will be None.")
                self.dmp = loaded_data
                self.gripper_trajectory = None

            if self.dmp:
                rospy.loginfo("DMP object loaded successfully.")
            else:
                 rospy.logerr("Failed to load DMP object.")

        except FileNotFoundError:
            rospy.logerr(f"DMP file not found: {filepath}")
            self.dmp = None
            self.gripper_trajectory = None
        except Exception as e:
            rospy.logerr(f"Error loading DMP data from {filepath}: {e}")
            self.dmp = None
            self.gripper_trajectory = None
    
    def compute_IK_trajectory(self, trajectory, time_stamp, q0=None, subsample_factor=1):
        if q0 is None:
            q0 = np.array([-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194])
        
        # Subsample the trajectory if requested
        if subsample_factor > 1:
            subsampled_trajectory = trajectory[::subsample_factor]
            subsampled_time_stamp = time_stamp[::subsample_factor]
            subsampled_gripper_trajectory = self.gripper_trajectory[::subsample_factor] if self.gripper_trajectory is not None else None
            print(f"Subsampled time from {len(time_stamp)} to {len(subsampled_time_stamp)} points")
            print(f"Subsampled trajectory from {len(trajectory)} to {len(subsampled_trajectory)} points")
        else:
            subsampled_trajectory = trajectory
            subsampled_time_stamp = time_stamp
            subsampled_gripper_trajectory = self.gripper_trajectory
        
        print(f"Solving inverse kinematics for {len(subsampled_trajectory)} points...")
        
        start_time = time.time()
        
        # Use the same random state as in dmp_test_1.py
        random_state = np.random.RandomState(0)
        joint_trajectory = self.chain.inverse_trajectory(
            subsampled_trajectory, random_state=random_state, orientation_weight=1.0)
            
        print(f"IK solved in {time.time() - start_time:.2f} seconds")
        
        return subsampled_trajectory, joint_trajectory, subsampled_gripper_trajectory, subsampled_time_stamp

    def visualize_trajectory(self, trajectory, joint_trajectory, q0=None):
        """
        Visualize the generated trajectory with optional subsampling
        
        Parameters:
        -----------
        trajectory : array-like
            The trajectory to visualize as homogeneous transformation matrices
        joint_trajectory : array-like
            Joint trajectory for animation
        q0 : array-like, optional
            Initial joint configuration for inverse kinematics
        """
        
        print(f"Plotting trajectory...")
        fig = pv.figure()
        fig.plot_transform(s=0.3)
        
        # Use the same whitelist as in dmp_test_1.py
        graph = fig.plot_graph(
            self.kin.tm, "world", show_visuals=False, show_collision_objects=True,
            show_frames=True, s=0.1, whitelist=[self.base_link, self.end_effector_link])

        # Plot start and end pose for clarity
        fig.plot_transform(trajectory[0], s=0.15)
        fig.plot_transform(trajectory[-1], s=0.15)
        
        # Always show the full trajectory in the visualization
        pv.Trajectory(trajectory, s=0.05).add_artist(fig)
        
        fig.view_init()
        fig.animate(
            animation_callback, len(trajectory), loop=True,
            fargs=(graph, self.chain, joint_trajectory))
        fig.show()


class GazeboTrajectoryPublisher:
    """Publisher for sending trajectories to Gazebo simulation using JointTrajectory messages"""
    
    def __init__(self, joint_names=None, gripper_joint_names=None):
        """
        Initialize Gazebo trajectory publisher
        
        Parameters:
        -----------
        joint_names : list, optional
            List of arm joint names
        gripper_joint_names : list, optional
            List of gripper joint names
        """
        if not rospy.core.is_initialized():
            rospy.init_node("gazebo_trajectory_publisher", anonymous=True)
        
        # Default joint names
        self.joint_names = joint_names or ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = gripper_joint_names or ["gripper", "gripper_sub"]
        
        # Publishers for arm and gripper
        self.arm_pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', 
                                     JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher('/open_manipulator_6dof/gripper_controller/command', 
                                         JointTrajectory, queue_size=10)
        
        print(f"[Gazebo] Initialized publishers:")
        print(f"  - Arm: /open_manipulator_6dof/arm_controller/command")
        print(f"  - Gripper: /open_manipulator_6dof/gripper_controller/command")
        
        # Wait for publishers to connect
        rospy.sleep(1.0)

    def publish_trajectory(self, joint_trajectory, gripper_trajectory, timestamps, execute_time_factor=1.0):
        """
        Publish complete trajectory to Gazebo simulation
        
        Parameters:
        -----------
        joint_trajectory : np.ndarray
            Joint trajectory for the arm (N, 6)
        gripper_trajectory : np.ndarray
            Gripper trajectory (N,) - single value per timestep
        timestamps : np.ndarray
            Timestamps for each point (N,)
        execute_time_factor : float
            Factor to scale execution time (1.0 = real-time, 0.5 = half speed, etc.)
        """
        if len(joint_trajectory) == 0:
            rospy.logwarn("[Gazebo] Empty trajectory provided")
            return
        
        print(f"[Gazebo] Publishing trajectory with {len(joint_trajectory)} points")
        
        # Create arm trajectory message
        arm_msg = JointTrajectory()
        arm_msg.header.stamp = rospy.Time.now()
        arm_msg.joint_names = self.joint_names
        
        # Create gripper trajectory message
        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.joint_names = self.gripper_joint_names
        
        # Add trajectory points
        for i in range(len(joint_trajectory)):
            # Arm trajectory point
            arm_point = JointTrajectoryPoint()
            arm_point.positions = joint_trajectory[i].tolist()
            arm_point.velocities = [0.0] * len(self.joint_names)
            arm_point.accelerations = [0.0] * len(self.joint_names)
            arm_point.time_from_start = rospy.Duration.from_sec(
                (timestamps[i] - timestamps[0]) * execute_time_factor
            )
            arm_msg.points.append(arm_point)
            
            # Gripper trajectory point (both gripper joints get same value)
            if gripper_trajectory is not None and i < len(gripper_trajectory):
                gripper_point = JointTrajectoryPoint()
                gripper_value = gripper_trajectory[i]
                gripper_point.positions = [-2.0*gripper_value, -2.0*gripper_value]  # Both joints
                gripper_point.velocities = [0.0, 0.0]
                gripper_point.accelerations = [0.0, 0.0]
                gripper_point.time_from_start = rospy.Duration.from_sec(
                    (timestamps[i] - timestamps[0]) * execute_time_factor
                )
                gripper_msg.points.append(gripper_point)
        
        # Publish trajectories
        print(f"[Gazebo] Publishing arm trajectory with {len(arm_msg.points)} points")
        self.arm_pub.publish(arm_msg)
        
        if gripper_trajectory is not None and len(gripper_msg.points) > 0:
            print(f"[Gazebo] Publishing gripper trajectory with {len(gripper_msg.points)} points")
            self.gripper_pub.publish(gripper_msg)
        else:
            print(f"[Gazebo] No gripper trajectory to publish")
        
        print(f"[Gazebo] Trajectory published successfully")

    def publish_single_trajectory(self, full_trajectory, timestamps, execute_time_factor=1.0):
        """
        Publish trajectory from combined arm+gripper array
        
        Parameters:
        -----------
        full_trajectory : np.ndarray
            Combined trajectory (N, 7) where last column is gripper
        timestamps : np.ndarray
            Timestamps for each point (N,)
        execute_time_factor : float
            Factor to scale execution time
        """
        if full_trajectory.shape[1] >= 6:
            arm_traj = full_trajectory[:, :6]  # First 6 columns for arm
            gripper_traj = full_trajectory[:, 6] if full_trajectory.shape[1] > 6 else None
            
            self.publish_trajectory(arm_traj, gripper_traj, timestamps, execute_time_factor)
        else:
            rospy.logwarn(f"[Gazebo] Invalid trajectory shape: {full_trajectory.shape}")


# -------------------------------------- Helper functions --------------------------------------# 
def animation_callback(step, graph, chain, joint_trajectory):
    """Animation callback for visualization"""
    chain.forward(joint_trajectory[step])
    graph.set_data()
    return graph

def save_trajectory_data(joint_trajectory, timestamps, filepath):
    """
    Save trajectory data to a pickle file

    Parameters:
    -----------
    joint_trajectory : np.ndarray
        Joint trajectory array (N, D)
    timestamps : np.ndarray
        Timestamps array (N,)
    filepath : str
        Path to save the pickle file
    """
    data = {
        'trajectory': joint_trajectory,
        'timestamps': timestamps
    }
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    print(f"[SAVE] Trajectory data saved to {filepath}")

def load_trajectory_data(filepath):
    """
    Load trajectory data from a pickle file

    Parameters:
    -----------
    filepath : str
        Path to load the pickle file

    Returns:
    --------
    joint_trajectory : np.ndarray
        Loaded joint trajectory
    timestamps : np.ndarray
        Loaded timestamps
    """
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    
    joint_trajectory = data['trajectory']
    timestamps = data['timestamps']
    print(f"[LOAD] Loaded trajectory from {filepath} (length={len(joint_trajectory)})")
    return joint_trajectory, timestamps

def interpolate_joint_trajectory(joint_traj, time_stamps, target_freq=20.0):
    """
    Interpolate joint trajectory to the target frequency

    Parameters:
    -----------
    joint_traj : np.ndarray
        Original joint positions (N, D)
    time_stamps : np.ndarray
        Original timestamps (N,)
    target_freq : float
        Target frequency in Hz

    Returns:
    --------
    interp_traj : np.ndarray
        Interpolated joint trajectory (M, D)
    new_timestamps : np.ndarray
        New timestamps (M,)
    """
    num_joints = joint_traj.shape[1]
    duration = time_stamps[-1] - time_stamps[0]
    num_samples = int(duration * target_freq)
    new_timestamps = np.linspace(time_stamps[0], time_stamps[-1], num_samples)
    
    interp_traj = np.zeros((num_samples, num_joints))
    for i in range(num_joints):
        interpolator = interp1d(time_stamps, joint_traj[:, i], kind='linear', fill_value="extrapolate")
        interp_traj[:, i] = interpolator(new_timestamps)
    
    return interp_traj, new_timestamps

def get_xyz_from_world_to_three():
    print("Getting blue cube position...")
    if not rospy.core.is_initialized():
        rospy.init_node('tf_xyz_fetcher', anonymous=True)
    listener = tf.TransformListener()
    try:
        # Wait until the transform becomes available
        print("Waiting for transform /world -> /blue_cube...")
        listener.waitForTransform('/world', '/blue_cube', rospy.Time(0), rospy.Duration(1.0))
        
        # Get the latest available transform
        trans, _ = listener.lookupTransform('/world', '/blue_cube', rospy.Time(0))
        print(f"Blue cube position: {trans}")
        return trans
    except Exception as e:
        print(f"Error getting transform: {e}")
        return None

# -------------------------------------- MAIN --------------------------------------# 
if __name__ == "__main__":
    # Example usage for Gazebo:
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    bag_path = '/root/catkin_ws/recordings/pick11.bag'

    # Initialize the motion generator
    dmp_gen = DMPMotionGenerator(
        urdf_path, 
        mesh_path,
        base_link="world"
    )

    # Learn from demonstration
    Y, transforms, joint_traj, gripper_traj = dmp_gen.learn_from_rosbag(
        bag_path, 
        '/gravity_compensation_controller/traj_joint_states'
    )
    
    # Save the learned DMP if needed
    dmp_path = '/root/catkin_ws/recordings/learned_pick_motion_11.pkl'
    dmp_gen.save_dmp(dmp_path)
    
    # Generate new trajectory
    new_start = dmp_gen.dmp.start_y.copy()
    new_goal = dmp_gen.dmp.goal_y.copy()
    print(f"Original goal: {new_goal}")

    trans_3 = get_xyz_from_world_to_three()
    print(f"Blue cube: {trans_3}")
    
    new_start[:3] += np.array([0.0, 0.0, 0.0])
    new_goal[:3] = np.array(trans_3) + np.array([+0.0, 0.0, +0.01])  # Modify position
    
    print(f"New goal: {new_goal}")
    # Generate trajectory  
    T, trajectory = dmp_gen.generate_trajectory(start_y=new_start, goal_y=new_goal)

    # Compute IK trajectory
    trajectory, IK_joint_trajectory, gripper_traj, T = dmp_gen.compute_IK_trajectory(
        trajectory, T, subsample_factor=1)
    
    # Apply moving average filter to smooth IK trajectory
    window_size = 25  # Adjust window size as needed
    if len(IK_joint_trajectory) > window_size:
        original_start = IK_joint_trajectory[0,:].copy()
        original_end = IK_joint_trajectory[-1,:].copy()

        smoothed_IK_joint_trajectory = np.zeros_like(IK_joint_trajectory)
        for i in range(IK_joint_trajectory.shape[1]): # For each joint
            smoothed_IK_joint_trajectory[:, i] = np.convolve(IK_joint_trajectory[:, i], np.ones(window_size)/window_size, mode='same')

        smoothed_IK_joint_trajectory[0,:] = original_start
        smoothed_IK_joint_trajectory[-1,:] = original_end

        half_window = window_size // 2
        for i in range(IK_joint_trajectory.shape[1]):
            # Blend start
            for j in range(half_window):
                alpha = j / float(half_window) # Linear blend
                smoothed_IK_joint_trajectory[j, i] = (1 - alpha) * original_start[i] + alpha * smoothed_IK_joint_trajectory[j, i]
            # Blend end
            for j in range(half_window):
                alpha = j / float(half_window) # Linear blend
                idx_from_end = len(IK_joint_trajectory) - 1 - j
                smoothed_IK_joint_trajectory[idx_from_end, i] = (1 - alpha) * original_end[i] + alpha * smoothed_IK_joint_trajectory[idx_from_end, i]


        IK_joint_trajectory = smoothed_IK_joint_trajectory
        print(f"Applied moving average filter with window size {window_size} to IK trajectory.")

    else:
        print(f"Trajectory too short (length {len(IK_joint_trajectory)}) for moving average window {window_size}. Skipping smoothing.")

    
    dmp_gen.visualize_trajectory(trajectory, IK_joint_trajectory)
    
    # Align trajectory lengths
    traj_length = min(IK_joint_trajectory.shape[0], len(gripper_traj) if gripper_traj is not None else IK_joint_trajectory.shape[0])
    IK_joint_trajectory = IK_joint_trajectory[:traj_length, :]
    
    if gripper_traj is not None:
        gripper_traj = gripper_traj[:traj_length]
        full_trajectory = np.hstack((IK_joint_trajectory, -gripper_traj.reshape(-1, 1)))
    else:
        # Add zero gripper values if no gripper trajectory
        gripper_traj = np.zeros(traj_length)
        full_trajectory = np.hstack((IK_joint_trajectory, -gripper_traj.reshape(-1, 1)))
    
    # Interpolate to desired frequency
    interpolated_traj, interpolated_time = interpolate_joint_trajectory(
        full_trajectory, T[:traj_length], target_freq=100.0)
    
    # Save trajectory data
    # save_trajectory_data(interpolated_traj, interpolated_time, 
    #                     "/root/catkin_ws/src/recordings/gazebo_interpolated_traj.pkl")

    # Publish to Gazebo simulation
    try:
        publisher = GazeboTrajectoryPublisher()
        
        # Wait a moment for publishers to be ready
        rospy.sleep(2.0)
        
        # Separate arm and gripper trajectories
        arm_trajectory = interpolated_traj[:, :6]
        gripper_trajectory = interpolated_traj[:, 6]
        
        # Publish trajectory (execute at normal speed)
        publisher.publish_trajectory(arm_trajectory, -gripper_trajectory, 
                                   interpolated_time, execute_time_factor=5)
        
        print("[Gazebo] Trajectory execution started...")
        
        # Keep the node alive to see the execution
        rospy.sleep(max(interpolated_time) + 2.0)
        plt.plot(interpolated_time, gripper_traj, label='gripper')
        plt.show() 
    except rospy.ROSInterruptException:
        print("[Gazebo] ROS publishing interrupted.")
    except Exception as e:
        print(f"[Gazebo] Error during trajectory execution: {e}")



    