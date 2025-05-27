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


try:
    import assignment_2.dmp_controller as dmp_controller
except ImportError:
    print("Error importing dmp_controller from assignment_2. Please check the import path.")
    sys.exit(1)

# Now we check if cubes are in the peg spaces
