#include "open_manipulator_6dof_controller/open_manipulator_position_gazebo_controller.h"

namespace open_manipulator_controller
{

OpenManipulatorPositionGazeboController::OpenManipulatorPositionGazeboController()
    : node_handle_(""), priv_node_handle_("~")
{
    // Get joint names from parameter server (expected to be set in launch file or yaml)
    if (!priv_node_handle_.getParam("joints", joint_names_))
    {
        ROS_ERROR("Position Gazebo Controller: Failed to get 'joints' parameter.");
        // Provide default names if parameter is not set, adjust as needed
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper", "gripper_sub"};
        ROS_WARN("Position Gazebo Controller: Using default joint names.");
    }

    if (joint_names_.empty())
    {
         ROS_ERROR("Position Gazebo Controller: No joint names specified.");
         ros::shutdown();
         return;
    }

    ROS_INFO("Position Gazebo Controller: Initializing...");
    initPublishers();
    initSubscriber();
    ROS_INFO("Position Gazebo Controller: Initialized successfully.");
}

void OpenManipulatorPositionGazeboController::initPublishers()
{
    ROS_INFO("Position Gazebo Controller: Setting up Gazebo command publishers...");
    for (const auto& joint_name : joint_names_)
    {
        // Construct topic name, e.g., "/open_manipulator_6dof/joint1_position/command"
        // Assuming the controller node runs within the robot's namespace (e.g., /open_manipulator_6dof)
        // If not, adjust the topic name accordingly.
        std::string topic_name = "/" + node_handle_.getNamespace() + "/" + joint_name + "_position/command";
        // Use advertise on the main node handle if topics are absolute or relative to root
        // Use advertise on priv_node_handle_ if topics are relative to the node's private namespace
        gazebo_goal_joint_position_pub_[joint_name] = node_handle_.advertise<std_msgs::Float64>(topic_name, 10);
        ROS_INFO("Advertising on topic: %s", topic_name.c_str());
    }
     ROS_INFO("Position Gazebo Controller: Gazebo command publishers set up.");
}

void OpenManipulatorPositionGazeboController::initSubscriber()
{
    ROS_INFO("Position Gazebo Controller: Setting up trajectory joint states subscriber...");
    // Subscribe to the specified topic. Adjust namespace if needed.
    std::string topic_name = "/gravity_compensation_controller/traj_joint_states";
    traj_joint_states_sub_ = node_handle_.subscribe(topic_name, 10,
                                                   &OpenManipulatorPositionGazeboController::trajJointStatesCallback, this);
    ROS_INFO("Subscribing to topic: %s", topic_name.c_str());
}

void OpenManipulatorPositionGazeboController::trajJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Directly publish Gazebo commands upon receiving a message
    publishGazeboCommands(msg);
}

void OpenManipulatorPositionGazeboController::publishGazeboCommands(const sensor_msgs::JointState::ConstPtr& msg)
{
    std_msgs::Float64 command_msg;

    // Create a map from name to position for efficient lookup
    std::map<std::string, double> name_to_position;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        // Check if the joint is one we are controlling
        if (gazebo_goal_joint_position_pub_.count(msg->name[i])) {
             name_to_position[msg->name[i]] = msg->position[i];
        }
    }

    // Publish commands for the joints specified in joint_names_
    for (const auto& joint_name : joint_names_)
    {
        // Check if the current message contains this joint
        if (name_to_position.count(joint_name))
        {
            command_msg.data = name_to_position[joint_name];
            if (gazebo_goal_joint_position_pub_.count(joint_name)) {
                 gazebo_goal_joint_position_pub_[joint_name].publish(command_msg);
            } else {
                 ROS_WARN_THROTTLE(5.0, "Position Gazebo Controller: Publisher for joint '%s' not found.", joint_name.c_str());
            }
        }
        else
        {
            // Optional: Warn if a controlled joint is missing in the message
            // ROS_WARN_THROTTLE(5.0, "Position Gazebo Controller: Joint '%s' not found in received JointState message.", joint_name.c_str());
        }
    }
}

} // namespace open_manipulator_controller

// Main function to run the node
int main(int argc, char **argv)
{
    // Initialize ROS node
    // The node name should be unique, e.g., "position_gazebo_controller"
    ros::init(argc, argv, "open_manipulator_position_gazebo_controller");

    // Create an instance of the controller
    open_manipulator_controller::OpenManipulatorPositionGazeboController controller;

    // Spin to process callbacks
    ros::spin();

    return 0;
}