﻿/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/***********************************************************
** Modified by Hae-Bum Jung
************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/open_manipulator_6dof_control_gui/qnode.hpp"
#include "../include/open_manipulator_6dof_control_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace robotis_manipulator;

namespace open_manipulator_control_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv),
  open_manipulator_actuator_enabled_(false),
  open_manipulator_is_moving_(false)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  // wait();
   ros::Duration(1.0).sleep();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"open_manipulator_6dof_control_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n("");

  // msg publisher
  open_manipulator_option_pub_ = n.advertise<std_msgs::String>("option", 10);
  open_manipulator_gui_button_pub_ = n.advertise<std_msgs::Bool>("button_clicked", 10);
  // msg subscriber
  open_manipulator_states_sub_          = n.subscribe("states", 10, &QNode::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_    = n.subscribe("joint_states", 10, &QNode::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = n.subscribe("kinematics_pose", 10, &QNode::kinematicsPoseCallback, this);
  open_manipulator_motion_states_sub_   = n.subscribe("motion_state", 10, &QNode::motionStatesCallback, this);
  // service client
  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_to_kinematics_pose_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
  goal_task_space_path_position_only_client_   = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_task_space_path_orientation_only_client_= n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_orientation_only");
  goal_task_space_path_from_present_client_    = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");

  goal_tool_control_client_       = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  set_actuator_state_client_      = n.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
  goal_drawing_trajectory_client_ = n.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory");

  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();
}

void QNode::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;

  if(msg->open_manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    open_manipulator_actuator_enabled_ = true;
  else
    open_manipulator_actuator_enabled_ = false;
}
void QNode::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper"))  temp_angle.at(6) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void QNode::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;

  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  Eigen::Quaterniond temp_orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  present_kinematics_position_ = temp_position;
  present_kinematics_orientation_ = temp_orientation;

  kinematics_pose_.pose = msg->pose;
}

void QNode::motionStatesCallback(const open_manipulator_motion::MotionState::ConstPtr &msg)
{
  open_manipulator_motion_state_ = msg->motion_state;
}

std::vector<double> QNode::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> QNode::getPresentKinematicsPos()
{
  return present_kinematics_position_;
}
Eigen::Quaterniond QNode::getPresentKinematicsOri()
{
  return present_kinematics_orientation_;
}
Eigen::Vector3d QNode::getPresentKinematicsOriRPY()
{
  present_kinematics_orientation_rpy_ = math::convertQuaternionToRPYVector(present_kinematics_orientation_);

  return present_kinematics_orientation_rpy_;
}
bool QNode::getOpenManipulatorMovingState()
{
  return open_manipulator_is_moving_;
}
bool QNode::getOpenManipulatorActuatorState()
{
  return open_manipulator_actuator_enabled_;
}

void QNode::setOption(std::string opt)
{
  std_msgs::String msg;
  msg.data = opt;
  open_manipulator_option_pub_.publish(msg);
}

void QNode::setButtonState(bool state)
{
  std_msgs::Bool msg;
  msg.data = state;
  open_manipulator_gui_button_pub_.publish(msg);
}

bool QNode::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);

  srv.request.path_time = path_time;

  if(goal_joint_space_path_to_kinematics_pose_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setTaskSpacePathPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setTaskSpacePathOrientationOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";
  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(2);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(3);

  srv.request.path_time = path_time;

  if(goal_task_space_path_orientation_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setTaskSpacePathFromPresent(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "gripper";
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);

  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setDrawingTrajectory(std::string name, std::vector<double> arg, double path_time)
{
  open_manipulator_msgs::SetDrawingTrajectory srv;
  srv.request.end_effector_name = "gripper";
  srv.request.drawing_trajectory_name = name;
  srv.request.path_time = path_time;
  for(int i = 0; i < arg.size(); i ++)
    srv.request.param.push_back(arg.at(i));

  if(goal_drawing_trajectory_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setActuatorState(bool actuator_state)
{
  open_manipulator_msgs::SetActuatorState srv;
  srv.request.set_actuator_state = actuator_state;

  if(set_actuator_state_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}


}  // namespace open_manipulator_control_gui
