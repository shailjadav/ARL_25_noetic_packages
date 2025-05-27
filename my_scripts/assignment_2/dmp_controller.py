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
from tf.transformations import quaternion_from_matrix # Added for manual PQS conversion
import numpy as np
import geometry_msgs.msg
import time
import pickle
from sensor_msgs.msg import JointState
from scipy.interpolate import interp1d
import threading # Added for lock

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
    # from pytransform3d.transformations import pqs_from_transform_matrix # REMOVED
    from movement_primitives.kinematics import Kinematics
    from movement_primitives.dmp import CartesianDMP
except ImportError as e:
    # Use rospy logger if available, otherwise print
    log_func = rospy.logerr if rospy.core.is_initialized() else print
    log_func(f"Failed to import dependencies: {e}. Make sure pytransform3d and movement_primitives are installed.")
    exit()

# --- Helper function for manual PQS conversion ---
def _manual_pqs_from_transform(transform_matrix):
    """
    Manually converts a 4x4 transformation matrix to a PQS vector [x,y,z,qw,qx,qy,qz].
    Uses tf.transformations.
    """
    position = transform_matrix[0:3, 3]
    # quaternion_from_matrix returns (qx, qy, qz, qw)
    q_tf = quaternion_from_matrix(transform_matrix)
    # PQS format requires (qw, qx, qy, qz) for the quaternion part
    quaternion_pqs = np.array([q_tf[3], q_tf[0], q_tf[1], q_tf[2]])
    return np.concatenate((position, quaternion_pqs))

# Add this helper function after the _manual_pqs_from_transform function
def get_cube_number_from_name(cube_name):
    """Extract the numeric part from a cube name (e.g. 'cube_3' -> 3)"""
    if not isinstance(cube_name, str) or not cube_name.startswith("cube_"):
        return -1
    try:
        return int(cube_name.split('_')[-1])
    except (ValueError, IndexError):
        return -1

# --- Renamed and Enhanced Class: ROSTrajectoryHandler ---
class ROSTrajectoryHandler(BaseROSTrajectoryPublisher):
    """
    Handles ROS trajectory publishing and subscribes to joint states.
    Inherits publishing capabilities from BaseROSTrajectoryPublisher.
    """
    def __init__(self, ordered_joint_names_for_publishing,
                 publish_topic_name='/gravity_compensation_controller/traj_joint_states',
                 publish_rate_hz=20,
                 subscribe_joint_state_topic="/joint_states"):
        """
        Initializes the publisher and subscriber.
        Does NOT call rospy.init_node().

        Args:
            ordered_joint_names_for_publishing (list): List of joint names in the order
                                                       expected by the publisher.
            publish_topic_name (str): Topic to publish trajectories to.
            publish_rate_hz (float): Rate for the publisher.
            subscribe_joint_state_topic (str): Topic to subscribe to for joint states.
        """
        # Initialize the publisher part using the parent class's logic
        # (Replicating relevant parts of BaseROSTrajectoryPublisher.__init__ as it's not calling super())
        self.publisher = rospy.Publisher(publish_topic_name, JointState, queue_size=10)
        self.joint_names = list(ordered_joint_names_for_publishing) # For publishing
        self.rate = rospy.Rate(publish_rate_hz)
        self.topic_name = publish_topic_name # For publishing
        rospy.loginfo(f"[ROS Handler] Publisher ready on topic '{self.topic_name}' at {publish_rate_hz}Hz")

        # Subscriber part
        self.joint_state_topic = subscribe_joint_state_topic
        self._latest_joint_data = {
            "name": [],
            "position": [],
            "velocity": [],
            "effort": []
        }
        self._joint_state_lock = threading.Lock()
        self._joint_name_to_index = {} # For quick lookup from subscriber

        # Store the order of joints this handler is primarily concerned with (for get_joint_states default)
        self.ordered_joint_names = list(ordered_joint_names_for_publishing)


        try:
            self.joint_state_subscriber = rospy.Subscriber(
                self.joint_state_topic,
                JointState,
                self._joint_state_callback,
                queue_size=1 # Process most recent message
            )
            rospy.loginfo(f"[ROS Handler] Subscribed to joint states on '{self.joint_state_topic}'")
        except Exception as e:
            rospy.logerr(f"[ROS Handler] Failed to subscribe to {self.joint_state_topic}: {e}")
            self.joint_state_subscriber = None


    def _joint_state_callback(self, msg):
        """Internal callback to update the latest joint states."""
        with self._joint_state_lock:
            self._latest_joint_data["name"] = list(msg.name)
            self._latest_joint_data["position"] = list(msg.position)
            if len(msg.velocity) == len(msg.name):
                self._latest_joint_data["velocity"] = list(msg.velocity)
            else: # Fill with zeros if velocity is not available or mismatched
                self._latest_joint_data["velocity"] = [0.0] * len(msg.name)
            if len(msg.effort) == len(msg.name):
                self._latest_joint_data["effort"] = list(msg.effort)
            else: # Fill with zeros if effort is not available or mismatched
                self._latest_joint_data["effort"] = [0.0] * len(msg.name)

            # Update lookup table
            self._joint_name_to_index = {name: i for i, name in enumerate(msg.name)}


    def get_joint_states(self, desired_joint_order=None):
        """
        Retrieves the latest joint states for specified joints in a desired order.

        Args:
            desired_joint_order (list of str, optional): A list of joint names in the
                desired order for the output. If None, uses the
                `ordered_joint_names` the class was initialized with.

        Returns:
            dict: A dictionary with keys 'name', 'position', 'velocity', 'effort'.
                  Each key maps to a list of values corresponding to `desired_joint_order`.
                  Returns None if no data is available or if subscriber failed.
                  If a desired joint is not found in the latest message, its values will be None.
        """
        if self.joint_state_subscriber is None and not self._latest_joint_data["name"]: # Check if subscriber failed and no data ever
            rospy.logwarn_throttle(5.0, "[ROS Handler] Cannot get joint states: subscriber not available or no data received.")
            return None

        with self._joint_state_lock:
            if not self._latest_joint_data["name"]: # No data received yet
                rospy.logwarn_throttle(5.0, "[ROS Handler] No joint state data received yet.")
                return None

            target_order = desired_joint_order if desired_joint_order is not None else self.ordered_joint_names

            ordered_positions = []
            ordered_velocities = []
            ordered_efforts = []

            for name in target_order:
                if name in self._joint_name_to_index:
                    idx = self._joint_name_to_index[name]
                    ordered_positions.append(self._latest_joint_data["position"][idx])
                    ordered_velocities.append(self._latest_joint_data["velocity"][idx])
                    ordered_efforts.append(self._latest_joint_data["effort"][idx])
                else:
                    rospy.logwarn_throttle(10.0, f"[ROS Handler] Joint '{name}' not found in latest joint state message. Appending None.")
                    ordered_positions.append(None)
                    ordered_velocities.append(None)
                    ordered_efforts.append(None)
            
            return {
                "name": list(target_order), # Ensure it's a list copy
                "position": ordered_positions,
                "velocity": ordered_velocities,
                "effort": ordered_efforts
            }

    # Inherits publish_trajectory from BaseROSTrajectoryPublisher


# --- Main Node Class ---
class DMPCubeManipulator:
    def __init__(self, dmp_paths, urdf_path, mesh_path=None,
                 base_frame="world",
                 robot_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                 gripper_joint_name="gripper",
                 publish_topic='/gravity_compensation_controller/traj_joint_states',
                 publish_rate=20.0,
                 tf_update_rate=5.0,
                 joint_states_topic="/joint_states",
                 cube_height_approx=0.04,
                 cube_xy_proximity_threshold=0.03,
                 use_sim = False): # Added joint_states_topic
        """
        Initializes the ROS node, TF listener, multiple DMP loaders, and trajectory handler.
        Args:
            # ... (dmp_paths, urdf_path etc. as before)
            joint_states_topic (str): Topic for subscribing to robot joint states.
        """
        if not rospy.core.is_initialized():
             rospy.init_node('dmp_cube_manipulator_node', anonymous=True)
        else:
             rospy.logwarn("ROS node 'dmp_cube_manipulator_node' already initialized.")

        rospy.loginfo("Initializing DMPCubeManipulator Node...")

        self.base_frame = base_frame
        self.robot_joint_names = list(robot_joint_names) # Ensure it's a list
        self.gripper_joint_name = gripper_joint_name
        self.all_joint_names_ordered = self.robot_joint_names + [self.gripper_joint_name]

        self.tf_listener = tf.TransformListener()
        self.tracked_cube_name = None
        self.current_cube_pose = None
        self.tf_update_rate = tf_update_rate
        self.tf_timer = None
        self.last_ee_pqs = None

        self.dmp_generators = {}
        required_motions = ['pick', 'lift', 'place', 'home']
        loaded_motions = []
        for motion_type, dmp_file in dmp_paths.items():
            rospy.loginfo(f"Loading DMP for motion '{motion_type}' from {dmp_file}...")
            generator = DMPMotionGenerator(
                urdf_path=urdf_path,
                mesh_path=mesh_path,
                joint_names=self.robot_joint_names, # DMP generator uses arm joints
                base_link=self.base_frame
            )
            try:
                generator.load_dmp(dmp_file)
                if generator.dmp is None:
                    raise RuntimeError(f"DMP object is None after loading from {dmp_file}")
                if motion_type in ['pick', 'lift', 'place'] and generator.gripper_trajectory is None:
                    rospy.logwarn(f"Gripper trajectory not found in DMP file for '{motion_type}'.")
                elif generator.gripper_trajectory is None:
                     rospy.loginfo(f"Gripper trajectory not found for '{motion_type}' (may be expected).")
                self.dmp_generators[motion_type] = generator
                loaded_motions.append(motion_type)
                rospy.loginfo(f"Successfully loaded DMP for '{motion_type}'.")
            except Exception as e:
                rospy.logerr(f"Failed to load DMP for motion '{motion_type}' from {dmp_file}: {e}")
                if motion_type in required_motions:
                    raise RuntimeError(f"Failed to load required DMP for '{motion_type}'.")

        missing_required = [m for m in required_motions if m not in loaded_motions]
        if missing_required:
             raise RuntimeError(f"Missing required DMPs for motions: {missing_required}")
        if not self.dmp_generators:
            rospy.logfatal("No DMPs were loaded successfully. Exiting.")
            raise RuntimeError("Failed to load any DMPs.")

        # Instantiate the new ROSTrajectoryHandler
        self.trajectory_handler = ROSTrajectoryHandler(
            ordered_joint_names_for_publishing=self.all_joint_names_ordered, # Publishes for arm + gripper
            publish_topic_name=publish_topic,
            publish_rate_hz=publish_rate,
            subscribe_joint_state_topic=joint_states_topic
        )
        self.publish_rate = publish_rate # Still useful for interpolation logic
        self.cube_height_approx = cube_height_approx
        self.cube_xy_proximity_threshold = cube_xy_proximity_threshold
        rospy.loginfo(f"DMPCubeManipulator initialized. Loaded DMPs for: {list(self.dmp_generators.keys())}")

    # --- TF Listener Methods (start, stop, update, get) remain the same ---
    def start_tf_listener(self, cube_name):
        if self.tf_timer is not None:
            if self.tracked_cube_name == cube_name:
                rospy.loginfo(f"TF listener already running for '{cube_name}'.")
                return
            else:
                rospy.loginfo(f"Switching TF listener from '{self.tracked_cube_name}' to '{cube_name}'.")
                self.stop_tf_listener()
        rospy.loginfo(f"Starting TF listener for '{cube_name}' at {self.tf_update_rate} Hz.")
        self.tracked_cube_name = cube_name
        self.current_cube_pose = None
        self.tf_timer = rospy.Timer(rospy.Duration(1.0 / self.tf_update_rate),
                                    self.update_single_cube_pose_callback,
                                    oneshot=False)

    def stop_tf_listener(self):
        if self.tf_timer is not None:
            rospy.loginfo(f"Stopping TF listener for '{self.tracked_cube_name}'.")
            self.tf_timer.shutdown()
            self.tf_timer = None
            self.tracked_cube_name = None
        else:
            rospy.loginfo("TF listener is not currently running.")

    def update_single_cube_pose_callback(self, event=None):
        if self.tracked_cube_name is None:
            rospy.logwarn_throttle(10.0, "TF update callback called but no cube is being tracked.")
            return
        try:
            self.tf_listener.waitForTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0))
            # print(f"TF update for '{self.tracked_cube_name}': Translation: {trans}")
            self.current_cube_pose = (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            if self.current_cube_pose is not None:
                 rospy.logwarn_throttle(5.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Using last known pose.")
            else:
                 rospy.logwarn_throttle(10.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Pose not yet available.")
        except Exception as e:
             rospy.logerr(f"Unexpected error getting transform for {self.tracked_cube_name}: {e}")

    def get_latest_cube_pose(self, cube_name):
        if cube_name != self.tracked_cube_name:
            rospy.logerr(f"Requested pose for '{cube_name}', but TF listener is tracking '{self.tracked_cube_name}'. Call start_tf_listener('{cube_name}') first.")
            return None
        if self.current_cube_pose is None:
             rospy.logwarn(f"Pose for tracked cube '{cube_name}' is not yet available.")
             return None
        return self.current_cube_pose
    

    # --- Core Motion Generation and Execution ---
    def _generate_dmp_trajectory(self, motion_type, start_pose_pqs=None, goal_pose_pqs=None, target_tf_frame=None, offset = np.array([0., 0., 0.01])):
        # ... (this method remains largely the same, uses the start_pose_pqs it's given)
        if motion_type not in self.dmp_generators:
            rospy.logerr(f"No DMP loaded for motion type '{motion_type}'. Cannot generate trajectory.")
            return None, None

        generator = self.dmp_generators[motion_type]
        dmp = generator.dmp

        # If start_pose_pqs is None here, generator.generate_trajectory will use dmp.start_y
        final_start_pqs = dmp.start_y.copy()
        if start_pose_pqs.any():
            final_start_pqs[:3] = start_pose_pqs[:3] # Use provided start position, keep orientation from DMP
        final_goal_pqs = dmp.goal_y.copy()

        if target_tf_frame and motion_type in ['pick', 'place']:
            latest_pose = self.get_latest_cube_pose(target_tf_frame)
            if latest_pose is None:
                rospy.logerr(f"No pose available for TF frame '{target_tf_frame}'. Cannot generate trajectory for '{motion_type}'.")
                return None, None
            trans, rot = latest_pose
            rospy.loginfo(f"Using TF pose for '{target_tf_frame}' as goal position for '{motion_type}': Translation={trans}")
            final_goal_pqs[:3] = np.array(trans)
            final_goal_pqs[:3] += offset # Add offset to the goal position

        rospy.loginfo(f"Generating '{motion_type}' trajectory:")
        if final_start_pqs is not None:
            rospy.loginfo(f"  Start PQS (effective): {np.round(final_start_pqs, 3)}")
        else:
            rospy.loginfo(f"  Start PQS (effective): Using DMP default start_y: {np.round(dmp.start_y,3)}")
        rospy.loginfo(f"  Goal PQS (effective):  {np.round(final_goal_pqs, 3)}")

        T_cartesian, cartesian_trajectory = generator.generate_trajectory(
            start_y=final_start_pqs, # Can be None, handled by generate_trajectory
            goal_y=final_goal_pqs
        )
        if T_cartesian is None or cartesian_trajectory is None:
            rospy.logerr(f"DMP trajectory generation failed for '{motion_type}'.")
            return None, None
        return T_cartesian, cartesian_trajectory


    def _compute_joint_trajectory(self, motion_type, T_cartesian, cartesian_trajectory, subsample_factor=1):
        # ... (this method remains the same)
        if motion_type not in self.dmp_generators:
            rospy.logerr(f"No DMP loaded for motion type '{motion_type}'. Cannot compute IK.")
            return None, None, None
        if cartesian_trajectory is None or T_cartesian is None:
            rospy.logerr(f"Cannot compute IK for None Cartesian trajectory for '{motion_type}'.")
            return None, None, None

        generator = self.dmp_generators[motion_type]
        _, joint_trajectory_arm, gripper_traj, T_ik = generator.compute_IK_trajectory(
            trajectory=cartesian_trajectory,
            time_stamp=T_cartesian,
            subsample_factor=subsample_factor
        )
        if joint_trajectory_arm is None:
             rospy.logerr(f"IK computation failed for '{motion_type}'.")
             return None, None, None
        if gripper_traj is None:
             rospy.logwarn(f"IK computation for '{motion_type}' did not return a gripper trajectory. Using default (0.0).")
             gripper_traj = np.zeros(len(joint_trajectory_arm))
        return joint_trajectory_arm, gripper_traj, T_ik

    def execute_motion(self, motion_type, start_pose_pqs=None, goal_pose_pqs=None,
                       target_tf_frame=None, wait_for_tf_sec=1.0, subsample_factor_ik=1):
        rospy.loginfo(f"--- Initialising motion sequence: '{motion_type}' ---")

        effective_start_pqs = start_pose_pqs

        if effective_start_pqs is None: # No explicit start from caller
            if self.last_ee_pqs is not None:
                # Use last commanded EE position with current DMP's default start orientation
                generator = self.dmp_generators[motion_type]
                dmp_default_start_orientation = generator.dmp.start_y[3:]
                last_ee_position = self.last_ee_pqs
                if motion_type in ['home']:
                    effective_start_pqs = np.concatenate((last_ee_position[:3], dmp_default_start_orientation))
                else:
                    effective_start_pqs = last_ee_position.copy() # Copy to avoid modifying the original
                # effective_start_pqs = np.concatenate((last_ee_position, dmp_default_start_orientation))
                rospy.loginfo(f"For '{motion_type}', using last commanded EE pos: {np.round(last_ee_position,3)} "
                              f"with last EE orientation: {np.round(dmp_default_start_orientation,3)}")
            else:
                # No explicit start, no last_ee_pqs: try to use current robot state via FK
                rospy.loginfo(f"For '{motion_type}', no explicit start_pose_pqs and no last_ee_pqs. Attempting to use current robot pose via FK.")
                current_joint_states_data = self.trajectory_handler.get_joint_states(desired_joint_order=self.robot_joint_names)
                if current_joint_states_data and all(p is not None for p in current_joint_states_data["position"]):
                    current_arm_positions = np.array(current_joint_states_data["position"])
                    # print(f"Current joint names: {current_joint_states_data['name']}")
                    # print(f"Current arm positions: {current_arm_positions}")
                    # Use any generator for FK, as kinematics should be the same
                    # (or have a dedicated self.kinematics_model if DMPMotionGenerator instances could vary significantly)
                    generator = self.dmp_generators[motion_type]
                    try:
                        current_ee_transform = generator.chain.forward(current_arm_positions)
                        # print(f"Current EE transform: {current_ee_transform}")
                        current_ee_pqs = _manual_pqs_from_transform(current_ee_transform) # USE MANUAL CONVERSION
                        # print(f"Current EE PQS: {current_ee_pqs}")
                        # generator = self.dmp_generators[motion_type] # Get current motion's generator
                        # dmp_default_start_orientation = generator.dmp.start_y[3:]
                        # current_ee_position = current_ee_pqs[:3]
                        effective_start_pqs = current_ee_pqs # Copy to avoid modifying the original
                        # effective_start_pqs = np.concatenate((current_ee_position, dmp_default_start_orientation))
                        rospy.loginfo(f"For '{motion_type}', using current FK EE pos: {np.round(effective_start_pqs,3)} ")
                    except Exception as e:
                        rospy.logwarn(f"Failed to get current EE pose via FK for '{motion_type}': {e}. DMP will use its default start_y.")
                        effective_start_pqs = None # Fallback to DMP default
                else:
                    rospy.logwarn(f"Could not retrieve valid current joint states for FK for '{motion_type}'. DMP will use its default start_y.")
                    effective_start_pqs = None # Fallback to DMP default
        else:
            rospy.loginfo(f"For '{motion_type}', using explicitly provided start_pose_pqs: {np.round(effective_start_pqs,3)}")

        goal_offset = np.array([0., 0., 0.01])

        needs_tf = target_tf_frame and motion_type in ['pick', 'place']
        if needs_tf:
            self.start_tf_listener(target_tf_frame)
            rospy.loginfo(f"Waiting {wait_for_tf_sec} seconds for TF data for '{target_tf_frame}'...")
            rospy.sleep(wait_for_tf_sec)

        if motion_type == 'place':
            goal_offset = np.array([-0.01, 0., 0.02]) # Adjust for place motion
        T_cartesian, cartesian_trajectory = self._generate_dmp_trajectory(
            motion_type, start_pose_pqs=effective_start_pqs, goal_pose_pqs=goal_pose_pqs, target_tf_frame=target_tf_frame, offset= goal_offset
        )
        if cartesian_trajectory is None:
            rospy.logerr(f"Failed to generate Cartesian trajectory for '{motion_type}'.")
            if needs_tf: self.stop_tf_listener()
            return None, None, None, None

        joint_trajectory_ik, gripper_traj, T_ik = self._compute_joint_trajectory(
            motion_type, T_cartesian, cartesian_trajectory, subsample_factor=subsample_factor_ik
        )

        if needs_tf:
            self.stop_tf_listener()

        if joint_trajectory_ik is None:
            rospy.logerr(f"Failed to compute joint trajectory via IK for '{motion_type}'.")
            return None, None, None, None

        if cartesian_trajectory is not None and len(cartesian_trajectory) > 0:
            last_transform_matrix = cartesian_trajectory[-1]
            self.last_ee_pqs = _manual_pqs_from_transform(last_transform_matrix) # USE MANUAL CONVERSION
            rospy.loginfo(f"Updated last_ee_pqs for next motion: {np.round(self.last_ee_pqs,3)}")

        rospy.loginfo(f"Successfully generated joint trajectory for '{motion_type}' with {len(joint_trajectory_ik)} points.")
        return joint_trajectory_ik, gripper_traj, T_ik, cartesian_trajectory

    # --- Specific Motion Wrappers (Convenience Functions) ---
    def pick_cube(self, cube_name, **kwargs):
        rospy.loginfo(f"Executing PICK motion for cube '{cube_name}'")
        if 'pick' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'pick_cube': 'pick' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='pick', target_tf_frame=cube_name, **kwargs)

    def lift_cube(self, **kwargs):
        rospy.loginfo(f"Executing LIFT motion")
        if 'lift' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'lift_cube': 'lift' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='lift', **kwargs)

    def place_cube(self, target_pose_pqs=None, target_tf=None, **kwargs):
        rospy.loginfo(f"Executing PLACE motion")
        if 'place' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'place_cube': 'place' DMP not loaded.")
            return None, None, None, None
        if target_tf:
            rospy.loginfo(f"  Targeting TF frame: {target_tf}")
            return self.execute_motion(motion_type='place', target_tf_frame=target_tf, **kwargs)
        elif target_pose_pqs is not None:
             rospy.loginfo(f"  Targeting PQS pose: {np.round(target_pose_pqs, 3)}")
             return self.execute_motion(motion_type='place', goal_pose_pqs=target_pose_pqs, **kwargs)
        else:
             rospy.logwarn("Executing 'place' motion with default DMP goal (no target specified).")
             return self.execute_motion(motion_type='place', **kwargs)

    def go_home(self, **kwargs):
        rospy.loginfo(f"Executing GO_HOME motion")
        if 'home' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'go_home': 'home' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='home', **kwargs)

    # --- Simulation and Publishing ---
    def simulate_trajectory(self, motion_type, joint_trajectory, cartesian_trajectory):
        # ... (remains the same)
        if motion_type not in self.dmp_generators:
            rospy.logwarn(f"Cannot simulate, no DMP generator for motion '{motion_type}'.")
            return
        if joint_trajectory is None or cartesian_trajectory is None:
            rospy.logwarn(f"Cannot simulate None trajectory for '{motion_type}'.")
            return
        generator = self.dmp_generators[motion_type]
        if len(joint_trajectory) != len(cartesian_trajectory):
             rospy.logwarn(f"Simulation Warning ({motion_type}): Joint trajectory length ({len(joint_trajectory)}) "
                           f"differs from Cartesian trajectory length ({len(cartesian_trajectory)}).")
        rospy.loginfo(f"Starting simulation for '{motion_type}' (requires graphical environment)...")
        try:
            generator.visualize_trajectory(cartesian_trajectory, joint_trajectory)
            rospy.loginfo("Simulation finished.")
        except Exception as e:
             rospy.logerr(f"Simulation failed for '{motion_type}': {e}.")


    def publish_trajectory(self, joint_trajectory_arm, timestamps, gripper_state):
        # ... (remains the same, uses self.trajectory_handler.publish_trajectory implicitly via inheritance)
        if joint_trajectory_arm is None or timestamps is None or gripper_state is None:
            rospy.logwarn("Cannot publish None trajectory components.")
            return

        num_points = len(joint_trajectory_arm)
        num_arm_joints = len(self.robot_joint_names)

        if joint_trajectory_arm.shape[1] != num_arm_joints:
             rospy.logerr(f"Arm trajectory dimension ({joint_trajectory_arm.shape[1]}) doesn't match robot joint names ({num_arm_joints}).")
             return

        full_joint_trajectory = np.zeros((num_points, num_arm_joints + 1))
        full_joint_trajectory[:, :num_arm_joints] = joint_trajectory_arm

        if isinstance(gripper_state, (int, float)):
            full_joint_trajectory[:, num_arm_joints] = gripper_state
            # rospy.loginfo(f"Applying constant gripper state: {gripper_state}") # Less verbose
        elif isinstance(gripper_state, np.ndarray):
            if len(gripper_state) == num_points:
                full_joint_trajectory[:, num_arm_joints] = gripper_state.flatten()
                # rospy.loginfo("Applying time-varying gripper state.") # Less verbose
            else:
                rospy.logwarn(f"Gripper state array length ({len(gripper_state)}) doesn't match traj length ({num_points}). Applying 0.0.")
                full_joint_trajectory[:, num_arm_joints] = 0.0
        else:
             rospy.logwarn(f"Invalid gripper_state type ({type(gripper_state)}). Applying 0.0.")
             full_joint_trajectory[:, num_arm_joints] = 0.0
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
        rospy.loginfo(f"Publishing interpolated trajectory ({len(interpolated_traj)} points) at {self.publish_rate} Hz...")
        try:
            # ROSTrajectoryHandler inherits publish_trajectory from BaseROSTrajectoryPublisher
            self.trajectory_handler.publish_trajectory(interpolated_traj, interpolated_time)
            rospy.loginfo("Trajectory publishing complete.")
        except Exception as e:
             rospy.logerr(f"Error during trajectory publishing: {e}")


    # Add this method to the DMPCubeManipulator class right before the run() method
    def check_hanoi_tower_condition(self, found_cubes):
        """
        Checks if cubes are stacked according to Tower of Hanoi rules.
        No cube with a higher number should be on top of a cube with a lower number.
        
        Args:
            found_cubes (list): List of cube TF names to check
        
        Returns:
            bool: True if no violations found, False otherwise
        """
        if not found_cubes or len(found_cubes) < 2:
            rospy.loginfo("Not enough cubes to check Tower of Hanoi condition.")
            return True
        
        # Get positions of all cubes
        cube_positions = {}
        for cube_name in found_cubes:
            try:
                self.tf_listener.waitForTransform(self.base_frame, cube_name, rospy.Time(0), rospy.Duration(0.1))
                (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, cube_name, rospy.Time(0))
                cube_number = get_cube_number_from_name(cube_name)
                if cube_number != -1:
                    cube_positions[cube_name] = {
                        'pos': trans, 
                        'number': cube_number
                    }
            except Exception as e:
                rospy.logwarn(f"Could not get transform for '{cube_name}': {e}")
        
        if len(cube_positions) < 2:
            return True  # Not enough valid cubes with positions
        
        # Check all pairs of cubes
        all_rules_satisfied = True
        for cube1_name, cube1_data in cube_positions.items():
            for cube2_name, cube2_data in cube_positions.items():
                if cube1_name == cube2_name:
                    continue
                    
                pos1, num1 = cube1_data['pos'], cube1_data['number']
                pos2, num2 = cube2_data['pos'], cube2_data['number']
                
                # Check if cubes are aligned in X-Y plane
                x_aligned = abs(pos1[0] - pos2[0]) < self.cube_xy_proximity_threshold
                y_aligned = abs(pos1[1] - pos2[1]) < self.cube_xy_proximity_threshold
                
                if x_aligned and y_aligned:
                    # Check if one is on top of another
                    z_diff = abs(pos1[2] - pos2[2])
                    if abs(z_diff - self.cube_height_approx) < (self.cube_height_approx / 2.0):
                        # We found a stack! Check the order
                        if pos1[2] > pos2[2]:  # cube1 is on top of cube2
                            if num1 > num2:
                                rospy.logwarn(f"Hanoi violation! Cube {cube1_name} (number {num1}) is on top of {cube2_name} (number {num2})")
                                all_rules_satisfied = False
                        elif pos2[2] > pos1[2]:  # cube2 is on top of cube1
                            if num2 > num1:
                                rospy.logwarn(f"Hanoi violation! Cube {cube2_name} (number {num2}) is on top of {cube1_name} (number {num1})")
                                all_rules_satisfied = False
        
        if all_rules_satisfied:
            rospy.loginfo("Tower of Hanoi condition is satisfied.")
        else:
            rospy.logwarn("Tower of Hanoi condition is violated!")
        
        return all_rules_satisfied

    def run(self):
        rospy.loginfo("DMPCubeManipulator node running. Waiting for commands or shutdown.")
        rospy.spin()



# -------------------------------------- MAIN Example --------------------------------------#
if __name__ == "__main__":
    grasper_node = None
    try:
        DMP_FILES = {
            'pick': '/root/catkin_ws/recordings/learned_pick_motion_11.pkl',
            'lift': '/root/catkin_ws/recordings/learned_lift_motion_4.pkl',
            'place': '/root/catkin_ws/recordings/learned_place_motion_14.pkl',
            'home': '/root/catkin_ws/recordings/learned_release_motion_2.pkl' # Renamed from 'home' in example
        }
        URDF_FILE = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
        MESH_DIR = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
        WORLD_FRAME = "world"
        CUBE_TO_GRASP = "red_cube" # Make sure this TF frame exists in your setup
        CUBE_TO_PLACE = "green_cube" 
        # PLACE_TARGET_POSE = np.array([0.2, 0.0, 0.05,  0.28606298 ,  0.05556376 , 0.94565966, -0.14425133]) # x,y,z,qw,qx,qy,qz
        # [0.20902298 -0.07639148  0.0803293   0.28606298  0.05556376  0.94565966 -0.14425133]
        LIFT_TARGET_PQS = np.array([0.08039667, -0.00823571, 0.12112987, 0.40824577, 0.03776871, 0.91182508, -0.02199852]) 

        TF_WAIT_TIME = 1.5
        IK_SUBSAMPLE = 5
        PUB_RATE = 20.0
        TF_RATE = 10.0
        PAUSE_BETWEEN_MOTIONS = 1.0 # Seconds
        JOINT_STATES_TOPIC = "/joint_states" # Default, but can be configured

        grasper_node = DMPCubeManipulator(
            dmp_paths=DMP_FILES,
            urdf_path=URDF_FILE,
            mesh_path=MESH_DIR,
            base_frame=WORLD_FRAME,
            tf_update_rate=TF_RATE,
            publish_rate=PUB_RATE,
            joint_states_topic=JOINT_STATES_TOPIC
        )

        # Allow some time for the joint state subscriber to receive first message
        rospy.loginfo("Waiting a moment for initial joint states...")
        rospy.sleep(1.0)


        # rospy.loginfo("Checking Tower of Hanoi condition...")
        # if  not grasper_node.check_hanoi_tower_condition(["cube_3", "cube_4"]):
        #     rospy.logwarn("Tower of Hanoi condition not satisfied. ")



        
        motion_sequence = [
            {'name': 'pick', 'action': lambda: grasper_node.pick_cube(CUBE_TO_GRASP, subsample_factor_ik=IK_SUBSAMPLE, wait_for_tf_sec=TF_WAIT_TIME)},
            {'name': 'lift', 'action': lambda: grasper_node.lift_cube(goal_pose_pqs=LIFT_TARGET_PQS, subsample_factor_ik=IK_SUBSAMPLE)},
            {'name': 'place', 'action': lambda: grasper_node.place_cube(target_tf=CUBE_TO_PLACE ,subsample_factor_ik=IK_SUBSAMPLE)},
            {'name': 'home', 'action': lambda: grasper_node.go_home(goal_pose_pqs=LIFT_TARGET_PQS, subsample_factor_ik=IK_SUBSAMPLE)} # Assuming 'home' DMP is for releasing/homing
        ]


        for motion_info in motion_sequence:
            rospy.loginfo(f"Executing: {motion_info['name']}")
            results = motion_info['action']()
            if results is None or results[0] is None: # results[0] is joint_traj_arm
                raise RuntimeError(f"{motion_info['name'].capitalize()} motion failed.")
            joint_traj_arm, gripper_traj, time_stamps, cartesian_traj_viz = results
            grasper_node.simulate_trajectory(motion_info['name'], joint_traj_arm, cartesian_traj_viz) # Optional
            # grasper_node.publish_trajectory(joint_traj_arm, time_stamps, gripper_traj) # UNCOMMENTED for actual execution
            rospy.loginfo(f"Waiting {PAUSE_BETWEEN_MOTIONS}s after {motion_info['name']}...")
            rospy.sleep(PAUSE_BETWEEN_MOTIONS)

        rospy.loginfo("Full motion sequence finished successfully.")
        if grasper_node: grasper_node.stop_tf_listener()

    except rospy.ROSInterruptException:
        rospy.loginfo("DMPCubeManipulator node interrupted.")
    except RuntimeError as e:
         rospy.logerr(f"A runtime error occurred during the motion sequence: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the main execution: {e}", exc_info=True)
    finally:
        if grasper_node is not None:
            rospy.loginfo("Ensuring TF listener is stopped in finally block.")
            grasper_node.stop_tf_listener()
        rospy.loginfo("DMPCubeManipulator node finished.")