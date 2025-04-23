#ifndef OPEN_MANIPULATOR_POSITION_GAZEBO_CONTROLLER_H
#define OPEN_MANIPULATOR_POSITION_GAZEBO_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <map>

namespace open_manipulator_controller
{

class OpenManipulatorPositionGazeboController
{
public:
    OpenManipulatorPositionGazeboController();
    virtual ~OpenManipulatorPositionGazeboController() = default;

private:
    // ROS NodeHandles
    ros::NodeHandle node_handle_;
    ros::NodeHandle priv_node_handle_;

    // ROS Subscriber
    ros::Subscriber traj_joint_states_sub_;

    // ROS Publishers (Map from joint name to publisher)
    std::map<std::string, ros::Publisher> gazebo_goal_joint_position_pub_;

    // Member variables
    std::vector<std::string> joint_names_; // List of joints to control

    // Initialization methods
    void initSubscriber();
    void initPublishers();

    // Callback function
    void trajJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // Helper function
    void publishGazeboCommands(const sensor_msgs::JointState::ConstPtr& msg);
};

} // namespace open_manipulator_controller

#endif // OPEN_MANIPULATOR_POSITION_GAZEBO_CONTROLLER_H