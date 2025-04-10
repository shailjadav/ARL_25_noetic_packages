# Open_manipulator_6dof_simulations

## [Simulation](#ros-simulation)

**NOTE**:
- This instruction has been tested on `Ubuntu 16.04` and `ROS Kinetic Kame`.
- This instruction is supposed to be run on PC with ROS packages installed in.

## [Launch gazebo](#launch-gazebo)
Load OpenManipulator on Gazebo simulator and click on Play `▶` button.

  ``` bash
  $ roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch
  ```

A red box is pointing end-effector link.
![](/images/simulation/Gazebo_1.png)

Enter `rostopic list` to list up the activated topics.

  ```
  /clock
  /gazebo/link_states
  /gazebo/model_states
  /gazebo/set_link_state
  /gazebo/set_model_state
  /open_manipulator_6dof/gripper/kinematics_pose
  /open_manipulator_6dof/gripper_position/command
  /open_manipulator_6dof/gripper_sub_position/command
  /open_manipulator_6dof/joint1_position/command
  /open_manipulator_6dof/joint2_position/command
  /open_manipulator_6dof/joint3_position/command
  /open_manipulator_6dof/joint4_position/command
  /open_manipulator_6dof/joint5_position/command
  /open_manipulator_6dof/joint6_position/command
  /open_manipulator_6dof/joint_states
  /open_manipulator_6dof/option
  /open_manipulator_6dof/states
  /rosout
  /rosout_agg
  ```

Open an [open_manipulator_6dof_control_gui](/images/GUI/GUI_start.png)

## [Controller for gazebo](#controller-for-gazebo)
Launch the open_manipulator_controller for gazebo simulation.

  ``` bash
  $ roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
  ```

**NOTE**:
- To control the OpenManipulator in the Gazebo environment using the OpenManipulator Controller, the controller must set the **use_platform** parameter to **false** because it needs to send messages to gazebo instead of Platform.
- If you want to manipulate the OpenManipulator using Moveit within the Gazebo simulator, you should also convert the **use_moveit** to **ture** in open_manipulator_6dof_controller launch file.

