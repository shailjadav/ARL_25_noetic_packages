#!/usr/bin/env python3
import sys
import os

# --- Path Setup ---
# Get the directory containing the current script (assignment_2)
current_script_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory (my_scripts)
parent_dir = os.path.dirname(current_script_dir)
# Add the parent directory to sys.path so Python can find assignment_1
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# --- Standard Imports ---
import rospy
import tf
import numpy as np
import geometry_msgs.msg
import time
import pickle
from sensor_msgs.msg import JointState
from scipy.interpolate import interp1d

# --- Imports from Sibling Directory (assignment_1) ---
try:
    from assignment_1.dmp_motions import (
        DMPMotionGenerator,
        ROSTrajectoryPublisher as BaseROSTrajectoryPublisher, # Import original as Base...
        interpolate_joint_trajectory,
        animation_callback # Import if needed for visualization
    )
except ImportError as e:
    print(f"ERROR: Failed to import from assignment_1.dmp_motions: {e}")
    print("Ensure assignment_1 directory and dmp_motions.py exist and parent directory is in sys.path.")
    print(f"Current sys.path includes: {parent_dir}")
    exit()

# --- Imports for DMP/Kinematics/Visualization ---
try:
    import pytransform3d.visualizer as pv
    import pytransform3d.trajectories as ptr
    from movement_primitives.kinematics import Kinematics
    from movement_primitives.dmp import CartesianDMP
except ImportError as e:
    # Use rospy logger if available, otherwise print
    log_func = rospy.logerr if rospy.core.is_initialized() else print
    log_func(f"Failed to import dependencies: {e}. Make sure pytransform3d and movement_primitives are installed.")
    exit()


# --- Create a Patched Publisher Class ---
class PatchedROSTrajectoryPublisher(BaseROSTrajectoryPublisher):
    """
    A version of ROSTrajectoryPublisher that inherits from the original
    but overrides the __init__ method to avoid calling rospy.init_node().
    This allows it to be used as a component within another ROS node.
    """
    def __init__(self, joint_names, topic_name='/gravity_compensation_controller/traj_joint_states', rate_hz=20):
        """
        Overridden __init__ that does NOT call rospy.init_node().
        It replicates the necessary setup from the parent class's __init__.
        """
        # --- Replicated setup from BaseROSTrajectoryPublisher.__init__ ---
        # *without* calling rospy.init_node()

        # 1. Create the publisher
        # Assumes rospy node is already initialized elsewhere
        self.publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

        # 2. Handle joint names
        # We expect the full list (arm + gripper) to be passed in.
        self.joint_names = list(joint_names) # Make a copy

        # Optional: Add a check based on expected usage (e.g., 7 joints)
        expected_joints = 7 # Example: 6 arm + 1 gripper
        if len(self.joint_names) != expected_joints:
            rospy.logwarn(f"PatchedROSTrajectoryPublisher expected {expected_joints} joint names, got {len(self.joint_names)}")

        # 3. Create the rate object for controlling publishing frequency
        self.rate = rospy.Rate(rate_hz)

        # 4. Store the topic name (useful for logging/debugging)
        self.topic_name = topic_name

        # 5. Log that this patched version is initialized
        rospy.loginfo(f"[ROS - Patched] Publisher ready on topic '{self.topic_name}' at {rate_hz}Hz")

    # --- Method Inheritance ---
    # All other methods (like publish_trajectory) are inherited from BaseROSTrajectoryPublisher


# --- Main Node Class ---
class DMPCubeManipulator:
    def __init__(self, dmp_path, urdf_path, mesh_path=None,
                 base_frame="world", # Removed cube_prefixes and cube_indices
                 robot_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                 gripper_joint_name="gripper",
                 publish_topic='/gravity_compensation_controller/traj_joint_states',
                 publish_rate=20.0,
                 tf_update_rate=5.0):
        """
        Initializes the ROS node, TF listener, DMP loader, and trajectory publisher.
        TF listening is started on demand.
        """
        # Initialize node only once
        if not rospy.core.is_initialized():
             rospy.init_node('dmp_cube_grasper_node', anonymous=True)
        else:
             rospy.logwarn("ROS node 'dmp_cube_grasper_node' already initialized.")

        rospy.loginfo("Initializing DMPCubeManipulator Node...")

        self.base_frame = base_frame
        self.robot_joint_names = robot_joint_names
        self.gripper_joint_name = gripper_joint_name
        self.all_joint_names = self.robot_joint_names + [self.gripper_joint_name] # For publisher

        self.tf_listener = tf.TransformListener()
        # Store pose only for the currently tracked cube
        self.tracked_cube_name = None
        self.current_cube_pose = None # Stores (trans, rot) tuple

        # Timer management
        self.tf_update_rate = tf_update_rate
        self.tf_timer = None # Handle for the active timer

        # Instantiate DMP generator
        self.dmp_generator = DMPMotionGenerator(
            urdf_path=urdf_path,
            mesh_path=mesh_path,
            joint_names=self.robot_joint_names,
            base_link=self.base_frame
        )
        # Load the specified DMP (which should include gripper trajectory)
        self.dmp_generator.load_dmp(dmp_path)
        if self.dmp_generator.dmp is None:
             rospy.logerr(f"Failed to load DMP from {dmp_path}. Exiting.")
             raise RuntimeError(f"Failed to load DMP from {dmp_path}")

        # Instantiate the PATCHED trajectory publisher
        self.trajectory_publisher = PatchedROSTrajectoryPublisher(
            joint_names=self.all_joint_names,
            topic_name=publish_topic,
            rate_hz=publish_rate
        )
        self.publish_rate = publish_rate

        rospy.loginfo(f"DMPCubeManipulator initialized. Base frame: '{self.base_frame}'. DMP loaded from: {dmp_path}")
        rospy.loginfo("TF listener ready. Call start_tf_listener('cube_name') to begin tracking.")


    def start_tf_listener(self, cube_name):
        """Starts listening for TF transforms for the specified cube."""
        if self.tf_timer is not None:
            if self.tracked_cube_name == cube_name:
                rospy.loginfo(f"TF listener already running for '{cube_name}'.")
                return
            else:
                rospy.loginfo(f"Switching TF listener from '{self.tracked_cube_name}' to '{cube_name}'.")
                self.stop_tf_listener() # Stop the previous timer

        rospy.loginfo(f"Starting TF listener for '{cube_name}' at {self.tf_update_rate} Hz.")
        self.tracked_cube_name = cube_name
        self.current_cube_pose = None # Reset pose when starting for a new cube

        # Create and start the timer
        self.tf_timer = rospy.Timer(rospy.Duration(1.0 / self.tf_update_rate),
                                    self.update_single_cube_pose_callback,
                                    oneshot=False) # False for periodic updates

    def stop_tf_listener(self):
        """Stops the currently active TF listener timer."""
        if self.tf_timer is not None:
            rospy.loginfo(f"Stopping TF listener for '{self.tracked_cube_name}'.")
            self.tf_timer.shutdown()
            self.tf_timer = None
            self.tracked_cube_name = None
            # Optionally clear the pose: self.current_cube_pose = None
        else:
            rospy.loginfo("TF listener is not currently running.")


    def update_single_cube_pose_callback(self, event=None):
        """Timer callback to update the pose of the currently tracked cube."""
        if self.tracked_cube_name is None:
            rospy.logwarn_throttle(10.0, "TF update callback called but no cube is being tracked.")
            return

        try:
            # Wait briefly for the transform to become available
            self.tf_listener.waitForTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0), rospy.Duration(0.1))
            # Get the latest transform
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0))
            self.current_cube_pose = (trans, rot)
            # rospy.loginfo_throttle(5.0, f"Updated pose for {self.tracked_cube_name}: t={trans}, r={rot}") # Optional debug log
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            if self.current_cube_pose is not None:
                 rospy.logwarn_throttle(5.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Using last known pose.")
            else:
                 rospy.logwarn_throttle(10.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Pose not yet available.")
            # Optionally clear the pose if it becomes unavailable:
            # self.current_cube_pose = None
        except Exception as e:
             rospy.logerr(f"Unexpected error getting transform for {self.tracked_cube_name}: {e}")


    def get_latest_cube_pose(self, cube_name):
        """Returns the latest known pose (trans, rot) if it's the currently tracked cube."""
        if cube_name != self.tracked_cube_name:
            rospy.logerr(f"Requested pose for '{cube_name}', but TF listener is tracking '{self.tracked_cube_name}'. Call start_tf_listener('{cube_name}') first.")
            return None
        if self.current_cube_pose is None:
             rospy.logwarn(f"Pose for tracked cube '{cube_name}' is not yet available.")
             return None
        return self.current_cube_pose

    def _generate_grasp_trajectory(self, cube_name, start_pose_pqs=None):
        """Internal method to generate Cartesian trajectory towards a cube."""
        # Get pose - relies on the listener being active for this cube
        latest_pose = self.get_latest_cube_pose(cube_name)
        if latest_pose is None:
            rospy.logerr(f"No pose available for '{cube_name}'. Cannot generate trajectory. Ensure TF listener is active and transform exists.")
            return None, None

        trans, rot = latest_pose

        rospy.loginfo(f"Target cube '{cube_name}' TF pose: Translation={trans}, Rotation={rot}")
        # Convert TF pose (trans, rot=(x,y,z,w)) to PQS format (pos, quat=(w,x,y,z))
        # Use the goal orientation from the loaded DMP as a base
        goal_pose_pqs = self.dmp_generator.dmp.goal_y.copy()
        goal_pose_pqs[:3] = np.array(trans) # Update only the position part

        # --- Optional: Add grasp offset ---
        # Example: grasp 5cm above the cube's origin, maintaining world Z orientation
        # offset_z = 0.05
        # goal_pose_pqs[2] += offset_z # Add offset to Z component
        # You might need more complex offset calculations depending on desired grasp orientation
        # ---------------------------------

        rospy.loginfo(f"Generating trajectory for '{cube_name}' towards goal PQS: {np.round(goal_pose_pqs, 3)}")

        # Generate trajectory using the DMP generator instance
        T_cartesian, cartesian_trajectory = self.dmp_generator.generate_trajectory(
            start_y=start_pose_pqs, # Use DMP default start if None
            goal_y=goal_pose_pqs
        )
        if T_cartesian is None or cartesian_trajectory is None:
            rospy.logerr("DMP trajectory generation failed.")
            return None, None
        return T_cartesian, cartesian_trajectory

    def _compute_joint_trajectory(self, T_cartesian, cartesian_trajectory, subsample_factor=1):
        """Internal method to compute joint trajectory using IK."""
        if cartesian_trajectory is None or T_cartesian is None:
            rospy.logerr("Cannot compute IK for None Cartesian trajectory.")
            return None, None, None

        # Compute IK using the DMP generator instance
        # This now uses the gripper trajectory stored within dmp_generator
        _, joint_trajectory_arm, gripper_traj, T_ik = self.dmp_generator.compute_IK_trajectory(
            trajectory=cartesian_trajectory,
            time_stamp=T_cartesian,
            subsample_factor=subsample_factor # Subsample for faster IK if needed
            # q0 can be specified if needed, otherwise uses default from compute_IK_trajectory
        )
        if joint_trajectory_arm is None:
             rospy.logerr("IK computation failed.")
             return None, None, None

        return joint_trajectory_arm, gripper_traj, T_ik

    def grasp_cube(self, cube_name, subsample_factor_ik=1, start_pose_pqs=None, wait_for_tf_sec=1.0):
        """
        Starts TF listener, generates and computes the joint trajectory to grasp the specified cube.

        Args:
            cube_name (str): The name of the cube frame (e.g., "cube_1").
            subsample_factor_ik (int): Factor to subsample Cartesian trajectory before IK.
            start_pose_pqs (list/np.array, optional): Custom start pose [x,y,z,qw,qx,qy,qz]. Defaults to DMP start.
            wait_for_tf_sec (float): Time in seconds to wait after starting listener for TF data.

        Returns:
            tuple: (joint_trajectory_arm, gripper_trajectory, timestamps, cartesian_trajectory)
                   or (None, None, None, None) if failed.
        """
        rospy.loginfo(f"--- Initiating grasp sequence for '{cube_name}' ---")

        # 1. Start TF listener for the target cube
        self.start_tf_listener(cube_name)
        rospy.loginfo(f"Waiting {wait_for_tf_sec} seconds for TF data for '{cube_name}'...")
        rospy.sleep(wait_for_tf_sec) # Give the listener time to receive at least one transform

        # 2. Generate Cartesian Trajectory (will use the pose updated by the listener)
        T_cartesian, cartesian_trajectory = self._generate_grasp_trajectory(cube_name, start_pose_pqs)
        if cartesian_trajectory is None:
            rospy.logerr("Failed to generate Cartesian trajectory.")
            self.stop_tf_listener() # Stop listener on failure
            return None, None, None, None

        # 3. Compute Joint Trajectory via IK
        joint_trajectory_ik, gripper_traj, T_ik = self._compute_joint_trajectory(
            T_cartesian, cartesian_trajectory, subsample_factor=subsample_factor_ik
        )

        # 4. Stop the TF listener (optional, depends if you need it running after)
        # self.stop_tf_listener()

        if joint_trajectory_ik is None:
            rospy.logerr("Failed to compute joint trajectory via IK.")
            self.stop_tf_listener() # Stop listener on failure
            return None, None, None, None

        rospy.loginfo(f"Successfully generated joint trajectory for '{cube_name}' with {len(joint_trajectory_ik)} points.")
        # Return the Cartesian trajectory as well for potential visualization
        return joint_trajectory_ik, gripper_traj, T_ik, cartesian_trajectory

    def simulate_trajectory(self, joint_trajectory, cartesian_trajectory):
        """Visualize the computed joint trajectory."""
        if joint_trajectory is None or cartesian_trajectory is None:
            rospy.logwarn("Cannot simulate None trajectory.")
            return

        # Check for potential length mismatch due to IK subsampling
        if len(joint_trajectory) != len(cartesian_trajectory):
             rospy.logwarn(f"Simulation Warning: Joint trajectory length ({len(joint_trajectory)}) "
                           f"differs from Cartesian trajectory length ({len(cartesian_trajectory)}). "
                           f"Visualization might appear jerky due to subsampling during IK.")
             # Option 1: Visualize only the subsampled points (might be jerky)
             # Option 2: Recompute IK without subsampling just for visualization (slower)
             # Option 3: Interpolate the joint trajectory to match Cartesian length (might be inaccurate)
             # For now, we proceed but warn the user.

        rospy.loginfo("Starting trajectory simulation (requires graphical environment)...")
        try:
            self.dmp_generator.visualize_trajectory(cartesian_trajectory, joint_trajectory)
            rospy.loginfo("Simulation finished.")
        except Exception as e:
             rospy.logerr(f"Simulation failed: {e}. Ensure pytransform3d visualization is correctly installed and configured.")


    def publish_trajectory(self, full_joint_trajectory, timestamps):
        """
        Interpolates and publishes the joint trajectory (including gripper).

        Args:
            full_joint_trajectory (np.ndarray): Joint trajectory for the robot arm (N, num_motors). This includes the gripper.
            timestamps (np.ndarray): Timestamps for the trajectory (N,).
        """
        # if joint_trajectory_arm is None or timestamps is None:
        #     rospy.logwarn("Cannot publish None trajectory.")
        #     return

        # num_points = len(joint_trajectory_arm)
        # num_arm_joints = len(self.robot_joint_names)

        # if joint_trajectory_arm.shape[1] != num_arm_joints:
        #      rospy.logerr(f"Arm trajectory dimension ({joint_trajectory_arm.shape[1]}) doesn't match robot joint names ({num_arm_joints}). Cannot publish.")
        #      return

        # # Prepare full trajectory including gripper
        # full_joint_trajectory = np.zeros((num_points, num_arm_joints + 1))
        # full_joint_trajectory[:, :num_arm_joints] = joint_trajectory_arm

        # # Add gripper state
        # if isinstance(gripper_state, (int, float)):
        #     full_joint_trajectory[:, num_arm_joints] = gripper_state # Constant gripper value
        #     rospy.loginfo(f"Applying constant gripper state: {gripper_state}")
        # elif isinstance(gripper_state, np.ndarray):
        #     if len(gripper_state) == num_points:
        #         full_joint_trajectory[:, num_arm_joints] = gripper_state # Time-varying gripper
        #         rospy.loginfo("Applying time-varying gripper state.")
        #     else:
        #         rospy.logwarn(f"Gripper state array length ({len(gripper_state)}) doesn't match trajectory length ({num_points}). Applying constant 0.0.")
        #         full_joint_trajectory[:, num_arm_joints] = 0.0
        # else:
        #      rospy.logwarn(f"Invalid gripper_state type ({type(gripper_state)}). Applying constant 0.0.")
        #      full_joint_trajectory[:, num_arm_joints] = 0.0


        # Interpolate to the desired publishing rate
        print(f"Shape fo timestamps: {timestamps.shape} and full trajectory {full_joint_trajectory.shape}" )
        try:
            interpolated_traj, interpolated_time = interpolate_joint_trajectory(
                full_joint_trajectory, timestamps, target_freq=self.publish_rate
            )
        except Exception as e:
             rospy.logerr(f"Interpolation failed: {e}. Cannot publish.")
             return

        if interpolated_traj is None or len(interpolated_traj) == 0:
             rospy.logerr("Interpolation resulted in empty trajectory. Cannot publish.")
             return

        # Publish using the publisher instance
        rospy.loginfo(f"Publishing interpolated trajectory ({len(interpolated_traj)} points) at {self.publish_rate} Hz...")
        try:
            self.trajectory_publisher.publish_trajectory(interpolated_traj, interpolated_time)
            rospy.loginfo("Trajectory publishing complete.")
        except Exception as e:
             rospy.logerr(f"Error during trajectory publishing: {e}")

    def run(self):
        """Keeps the node running."""
        rospy.loginfo("DMPCubeManipulator node running. Waiting for grasp commands or shutdown.")
        rospy.spin() # Keep node alive until shutdown


# -------------------------------------- MAIN Example --------------------------------------#
if __name__ == "__main__":
    grasper_node = None # Initialize for finally block
    try:
        # --- Configuration ---
        # !! ADJUST THESE PATHS AND NAMES !!
        DMP_FILE = '/root/catkin_ws/recordings/learned_pick_motion.pkl' # Ensure this includes gripper data
        URDF_FILE = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
        MESH_DIR = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
        WORLD_FRAME = "world"
        CUBE_TO_GRASP = "cube_5" # Make sure this TF frame exists (e.g., from Gazebo or perception)
        TF_WAIT_TIME = 1.5 # Seconds to wait for TF after starting listener
        IK_SUBSAMPLE = 5 # Subsample factor for IK (1 = no subsampling)
        PUB_RATE = 20.0 # Hz for publishing trajectory
        TF_RATE = 10.0 # Hz for TF listener when active

        # --- Initialize Node ---
        grasper_node = DMPCubeManipulator(
            dmp_path=DMP_FILE,
            urdf_path=URDF_FILE,
            mesh_path=MESH_DIR,
            base_frame=WORLD_FRAME,
            tf_update_rate=TF_RATE,
            publish_rate=PUB_RATE
            # Other args use defaults
        )

        # --- Example Usage ---
        # No need for initial sleep here, grasp_cube handles starting the listener and waiting

        # 1. Generate trajectory for the target cube
        # grasp_cube now returns: joint_traj_arm, gripper_traj, time_stamps, cartesian_trajectory
        joint_traj_arm, gripper_traj, time_stamps, cartesian_traj_viz = grasper_node.grasp_cube(
            CUBE_TO_GRASP,
            subsample_factor_ik=IK_SUBSAMPLE,
            wait_for_tf_sec=TF_WAIT_TIME
        )
        print(f"Length of gripper trajectory: {len(gripper_traj)} and length of joint trajectory: {len(joint_traj_arm)}")
        traj_length = min(joint_traj_arm.shape[0], gripper_traj.shape[0])
        gripper_traj = gripper_traj[:traj_length]
        joint_traj_arm = joint_traj_arm[:traj_length,:]
        full_trajectory = np.hstack((joint_traj_arm, gripper_traj.reshape(-1, 1)))
        time_stamps = time_stamps[:traj_length]
        # Optional: Stop the listener explicitly if grasp_cube doesn't, or if done with TF
        # grasper_node.stop_tf_listener()

        if joint_traj_arm is not None:
            rospy.loginfo(f"Generated grasp trajectory for {CUBE_TO_GRASP}.")

            # 2. Simulate (Optional) - Requires graphical environment
            if cartesian_traj_viz is not None:
                 grasper_node.simulate_trajectory(joint_traj_arm, cartesian_traj_viz)
            else:
                 rospy.logwarn("Cannot simulate trajectory as Cartesian trajectory was not returned.")


            # 3. Publish the trajectory
            # rospy.loginfo("Publishing the generated trajectory...")

            # # Use the time-varying gripper trajectory obtained from IK/DMP loading
            # grasper_node.publish_trajectory(full_trajectory, time_stamps)

            rospy.loginfo("Grasp sequence finished.")
        else:
            rospy.logerr(f"Failed to generate grasp trajectory for {CUBE_TO_GRASP}.")

        # Stop listener if not already stopped
        grasper_node.stop_tf_listener()

        # Keep the node alive if needed for other tasks, e.g., if it provides services
        # rospy.loginfo("Node finished example task. Spinning...")
        # grasper_node.run() # Uncomment if you want the node to persist

    except rospy.ROSInterruptException:
        rospy.loginfo("DMPCubeManipulator node interrupted.")
    except RuntimeError as e:
         # Catch specific errors like DMP loading failure
         rospy.logerr(f"A runtime error occurred: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the main execution: {e}", exc_info=True) # Log traceback
    finally:
        # Ensure listener is stopped on exit, even if errors occurred
        if grasper_node is not None:
            rospy.loginfo("Ensuring TF listener is stopped in finally block.")
            grasper_node.stop_tf_listener()
        rospy.loginfo("DMPCubeManipulator node finished.")