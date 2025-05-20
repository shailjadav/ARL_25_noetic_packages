# OpenManipulator Sara for 'Advanced Methods of robot learning'



![Gravity Compensation Mode](fig/manipulation.gif)

## Update Packages within docker
In case you need to update the packages within docker, please follow these instructions:
```bash
cd /root/catkin_ws/ && rm -rf src \
mkdir src
git clone https://github.com/shailjadav/ARL_25_noetic_packages.git /root/catkin_ws/src/ \
catkin_make
```
If you face issues, have a look at the Troubleshooting and Known Issues sections of this ReadMe.


## Basic Usage
If you apply changes to the packages in the src folder please remember to build again
```bash
catkin_make
```
Occasionally some settings are cached in devel and build. Consider to delete both folder if building does not work appropriately.
After building the first thing to do in any new terminal session inside the container is to source the setup file. 

```bash
source /catkin_ws/devel/setup.bash
```

### Running in Simulation
This shows the deployment of the base controller (build by the team of Robotis)
1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** in a new terminal:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
   ```

3. **Launch Gazebo simulation** in another terminal:
   ```bash
   roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch controller:=position
   ```

4. **Launch the GUI control panel**:
   ```bash
   roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
   ```

5. **Run example scripts**:
   ```bash
   cd src/open_manipulator_friends/open_manipulator_6dof_controller/scripts/
   python example.py
   ```

### Running on Real Robot

1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** with hardware communication:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=true dynamixel_usb_port:=/dev/ttyUSB0
   ```
   Note: Adjust the USB port if your device is connected to a different port.

3. **Launch the GUI control panel (after homing stop it)**:
   ```bash
   roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
   ```

## Recording and Playback

### Preparing for Recording

1. Ensure the robot is in a known position before recording.

2. Use the change_mode script in the `my_scripts` folder. You can pass it an argument mode for chossing between position(3) mode and torque mode(0). For gravity compensation mode we need torque mode. For the om_position controller, we need position mode.
   ```bash
   roslaunch python3 /root/catkin_ws/src/my_scripts/change_mode.py --mode 0
   ```

3. Launch the gravity compensation controller:
   ```bash
   roslaunch open_manipulator_controllers gravity_compensation_controller.launch sim:=false
   ```

   This should enable the robot to float in the air like in this example:
   ![Gravity Compensation Mode](fig/gravity_compensation.gif)

4. Start end-effector position republishing:
   ```bash
   cd src/open_manipulator_friends/open_manipulator_6dof_controller/scripts/
   python republish_pose.py
   ```

5. Verify position data is being published:
   ```bash
   rostopic echo /current_pose
   ```

### Recording Movements

1. Create a directory for recordings (if not exists):
   ```bash
   mkdir -p ~/catkin_ws/recordings
   ```

2. Start recording all topics to a ROS bag:
   ```bash
   rosbag record -a -O ~/catkin_ws/recordings/move1.bag
   ```

3. Move the robot as desired (manually or through other control methods).

4. Press Ctrl+C to stop recording.

### Examining and Playing Back Recordings

1. View information about the recorded bag:
   ```bash
   rosbag info ~/catkin_ws/recordings/move1.bag
   ```

2. Play back the recorded movements:
   ```bash
   rosbag play ~/catkin_ws/recordings/move1.bag
   ```
3. Running on robot
   ```bash
    roslaunch om_position_controller position_control.launch
    rosbag play move1.bag --hz=50
   ```

# Assignments
In the folder `root/catkin_ws/src/my_scripts/` there is inspiration for solving the assignments. These are not fully functional solutions, but rather blueprints.
They are not here to solve your assignments, but rather point you to some direction.  

## Training and executing DMPs

Wihtin the folder`root/catkin_ws/src/my_scripts/assignment_1` there are scripts for
- checking the recorded motions with `motions_test.py`
- training DMPs using the recorded trajectories with `dmp_motions.py`. 

Building on top of these within the folder`root/catkin_ws/src/my_scripts/assignment_1` there are scripts for
- training YOLO on custom objects using the annotation of the initila frame and deploying SAM2 with the scripts `1_dataset_create.py`, `2_train_model.py` and `3_validate_model.py`.
- For detecting the objects using the realsense camera you can use `4_rs_detect.py`.
- Once Objects are detected and their position is evaluated in the world frame, you can use `dmp_controller.py` to use your learned motions and execute them onto custom target goals and start points.



## Installing Python packages
As the solutions for the tasks are not provided beforehand you need to install packages yourself for the solutions.
For the assignments in the `my_scripts` you can use the requirement.txt or install the packages by hand. The most relevant package here is `pip install movement_primitives[all]`.
Depending on your setup the python packages vary.

## Troubleshooting

- If Gazebo does not display, run `xhost +local:root` on the host machine.
- Check USB permissions if the robot is not detected.
- Ensure all dynamixel motors are in the correct control mode before recording.
- Verify the USB port in controller launch file matches your hardware setup.
- If there is an issue with em / empy. Do pip install empy=3.3.2

## Known issues

- If roscore does not start appropriately consider to check /etc/hosts it most probably looks like this:

```
127.0.0.1	localhost
::1		localhost ip6-localhost ip6-loopback
ff02::1		ip6-allnodes
ff02::2		ip6-allrouters
```
In some cases you may need to adapt the localhost to the hostname. To figure out the hostname type in the terminal inside the docker `hostname`

- If gazebo is not responding to any of the position commands try restarting the docker container.


For further issues please open an issue on GitHub. Please keep in mind that this repo is under current development and can not provide full scale support to all platforms such as Windows, Linux and MacOS. This repo is tested on a host system with Ubuntu 22.0.4.
 

## Acknowledgments

- ROBOTIS for the Open Manipulator platform
- ROS community for the control frameworks


