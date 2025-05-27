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
3. Running on robot or in simulation
   For executing in simulation please pass argunment sim:=true
   ```bash
    roslaunch om_position_controller position_control.launch sim:=false
    rosbag play move1.bag --hz=50
   ```


# Assignments

In the folder `root/catkin_ws/src/my_scripts/`, you’ll find reference scripts to help guide your assignment work. These are *not* complete solutions, but rather blueprints to point you in the right direction.

---

## Training and Executing DMPs

Within `root/catkin_ws/src/my_scripts/assignment_1`, you’ll find scripts for:

- **Visualizing recorded motions:**  
  `motions_test.py` — to inspect recorded trajectories.

- **Training DMPs using recorded trajectories:**  
  `dmp_motions.py` — to generate motion primitives from demonstrations.

---

## Object Detection and Motion Execution

In `root/catkin_ws/src/my_scripts/assignment_2`, you’ll build upon the previous assignment with object detection and motion control:

- **Custom object detection using YOLO and SAM2:**  
  Use the following scripts:
  - `1_dataset_create.py` – create dataset from the initial frame.
  - `2_train_model.py` – train the YOLO model.
  - `3_validate_model.py` – validate the trained model.

- **RealSense camera-based detection:**  
  - `4_rs_detect.py` – detect objects and determine their positions in the world frame.

- **Motion execution using DMPs:**  
  - `dmp_controller.py` – execute learned motions toward custom goals based on detected object positions.

---

## Simulated Robot Execution

If you’re unable to access or work with the real robot, a simulation environment is provided via the `om_position_controller` package. Follow these steps:

1. **Start the position controller (simulation mode):**
   ```bash
   roslaunch om_position_controller position_control.launch sim:=true
   ```

2. **Start the republishing of the cube coordinates**
   ```bash 
   roscd om_position_controller/scripts && \
   python3 4_rs_detect_sim.py 
   ```

3. **(Optional)Visualise your recordings**
   First play the rosbag and then execute this script
   ```bash 
   roscd om_position_controller/scripts && \
   python3 simulated_trajectory.py
   ```

4. **Execute the pick_and_place script.**
   ```bash 
   roscd om_position_controller/scripts && \
   python3 pick_and_place.py
   ```
![Simulation pick and place](fig/manipulation_in_sim.gif)


For going back to the home pose execute:
   ```bash 
   roscd om_position_controller/scripts && \
   python3 move_to_home.py 
   ```

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


