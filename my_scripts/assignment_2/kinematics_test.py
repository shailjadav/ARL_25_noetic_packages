#!/usr/bin/env python3
import numpy as np
import sys
import os

# --- Path Setup for assignment_1 imports ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_script_dir) # This should be my_scripts
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from assignment_1.dmp_motions import DMPMotionGenerator
except ImportError as e:
    print(f"ERROR: Failed to import DMPMotionGenerator from assignment_1.dmp_motions: {e}")
    print(f"Ensure assignment_1 directory and dmp_motions.py exist and parent directory ({parent_dir}) is in sys.path.")
    exit()

from tf.transformations import quaternion_from_matrix # For manual PQS conversion

# --- Helper function for manual PQS conversion (copied from dmp_controller.py) ---
def _manual_pqs_from_transform(transform_matrix):
    """
    Manually converts a 4x4 transformation matrix to a PQS vector [x,y,z,qw,qx,qy,qz].
    Uses tf.transformations.
    """
    position = transform_matrix[0:3, 3]
    q_tf = quaternion_from_matrix(transform_matrix) # (qx, qy, qz, qw)
    quaternion_pqs = np.array([q_tf[3], q_tf[0], q_tf[1], q_tf[2]]) # (qw, qx, qy, qz)
    return np.concatenate((position, quaternion_pqs))

if __name__ == "__main__":
    # --- Configuration ---
    URDF_FILE = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    
    # Base frame for the DMPMotionGenerator (as used in dmp_controller.py)
    BASE_FRAME = "world" 
    
    # Robot joint names (arm only, as DMPMotionGenerator takes these for its chain)
    # This must match the joints controlled by the 'joint_configuration'
    ROBOT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # Given joint configuration (for the arm joints specified in ROBOT_JOINT_NAMES)
    joint_configuration = np.array([
        -0.06289321184158325, 
        -1.0339031219482422, 
        1.5784662961959839, 
        -0.02454369328916073, 
        1.7548741102218628, 
        0.004601942375302315
    ])

    print(f"URDF File: {URDF_FILE}")
    print(f"Base Frame for DMPGenerator: {BASE_FRAME}")
    print(f"Robot Joint Names for DMPGenerator: {ROBOT_JOINT_NAMES}")
    print(f"Input Joint Configuration: {joint_configuration}")

    if len(joint_configuration) != len(ROBOT_JOINT_NAMES):
        print(f"Error: Length of joint_configuration ({len(joint_configuration)}) "
              f"does not match length of ROBOT_JOINT_NAMES ({len(ROBOT_JOINT_NAMES)}).")
        exit()

    try:
        # Initialize DMPMotionGenerator
        # We don't need to load a DMP, just need its kinematic chain setup.
        # The 'mesh_path' is optional for DMPMotionGenerator if not visualizing.
        dmp_generator = DMPMotionGenerator(
            urdf_path=URDF_FILE,
            joint_names=ROBOT_JOINT_NAMES,
            base_link=BASE_FRAME,
            # mesh_path can be omitted if not needed for this specific test
        )

        if dmp_generator.chain is None:
            print("Error: DMPMotionGenerator failed to initialize its kinematic chain.")
            exit()
            
        print(f"DMPMotionGenerator initialized. Kinematic chain created.")
        # print(f"Chain active links (from ikpy): {[link.name for link in dmp_generator.chain.links if link.joint_type != 'fixed']}")


        # Compute forward kinematics using the chain.forward() method
        # This is exactly how it's done in dmp_controller.py:
        # current_ee_transform = generator.chain.forward(current_arm_positions)
        ee_transform_matrix = dmp_generator.chain.forward(joint_configuration)

        print("\nEnd-Effector Transformation Matrix:")
        print(ee_transform_matrix)

        # Convert to PQS pose [x, y, z, qw, qx, qy, qz]
        ee_pqs_pose = _manual_pqs_from_transform(ee_transform_matrix)
        
        print("\nEnd-Effector PQS Pose (x, y, z, qw, qx, qy, qz):")
        print(ee_pqs_pose)

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        print("Please ensure the URDF path and other parameters are correct, "
              "and that the 'assignment_1.dmp_motions' module is accessible and functioning.")
