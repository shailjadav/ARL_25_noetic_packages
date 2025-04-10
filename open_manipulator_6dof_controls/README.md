# OpenMANIPULATOR Controls [![Build Status](https://travis-ci.org/ROBOTIS-GIT/open_manipulator_controls.svg?branch=master)](https://travis-ci.org/ROBOTIS-GIT/open_manipulator_controls)

## How to Run
```bash
(MoveGroup + JointTrajectoryController)
# Gazebo Simulation
roslaunch open_manipulator_controllers joint_trajectory_controller.launch

# Real Robot
roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false

(GravityCompensationController)
# Gazebo Simulation
# Set trasmission to effort and motor mode to current first
roslaunch open_manipulator_controllers gravity_compensation_controller.launch 

# Real Robot
roslaunch open_manipulator_controllers gravity_compensation_controller.launch sim:=false
```


## Hint for the exercise
Be sure to switch every dynamixel motor to torque mode before executing the controllers of this package.

Once you start the gravity compensation controller, you are able to record trajectories. This can be done using rosbag.
Simply start the recording with the command:
```
rosbag record -O trajectory.bag /gravity_compensation_controller/traj_joint_states
```
This will store the trajectory in a rosbag format.
For replay capabilities please refer to the [position based controller](https://github.com/DarioRepoRuler/om_position_controller.git) 