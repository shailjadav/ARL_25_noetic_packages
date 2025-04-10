/*******************************************************************************
* Impedance Control for 6-DOF Robotic Manipulation
* 
* Description:
* This implementation of Impedance Control is specifically tailored for a 6-DOF robotic 
* manipulator, focusing on achieving compliant behavior through force feedback control. 
* The code structure and computation methodologies were initially inspired by and adapted 
* from existing gravity compensation strategies employed previously.
*
* Extended from 4-DOF implementation to 6-DOF system.
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


#include "advanced_robotics_control/impedance_control_controller.h"

namespace advanced_robotics_control
{
bool ImpedanceControlController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Joint interface initialization
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr)
  {
    ROS_ERROR(
        "[ImpedanceControlController] Could not get effort joint interface "
        "from hardware!");
    return false;
  }

  // Do not set desired position yet - we'll get it from current position when starting
  // This prevents initial large errors
  use_current_as_desired_ = true;
  
  // Initialize stiffness and damping with very conservative values
  K_trans_ = KDL::Vector(40.0, 40.0, 40.0);  // Very low stiffness initially
  K_rot_ = KDL::Vector(0.0, 0.0, 0.0);    // Very low rotational stiffness
  
  // Higher damping for stability
  Kd_lin_ = KDL::Vector(0.1, 0.1, 0.1); // High translational damping
  Kd_ang_ = KDL::Vector(0.0, 0.0, 0.0);    // High rotational damping

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not parse joint names");
    return false;
  }
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    try
    {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "[ImpedanceControlController] Could not get joint handle: "
          << e.what());
      return false;
    }
  }

  // Initialize publishers
  torque_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("torque_values", 10);
  jacobian_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("jacobian_values", 10);
  end_effector_pos_publisher_ = node_handle.advertise<geometry_msgs::Point>("end_effector_position", 10);
  velocity_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("joint_velocities", 10);
  end_eff_velocity_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("end_effector_velocities", 10);
  end_effector_orient_euler_publisher_ = node_handle.advertise<geometry_msgs::Vector3>("end_effector_orientation", 10);

  // Optional subscriber to update desired pose at runtime
  desired_pose_subscriber_ = node_handle.subscribe("desired_pose", 1, &ImpedanceControlController::desiredPoseCallback, this);

  // KDL setup
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("[ImpedanceControlController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not construct kdl tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not find root link name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not find tip link name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR(
        "[ImpedanceControlController] Could not get KDL chain from tree");
    return false;
  }

  // After setting up the kdl_chain_
  jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(kdl_chain_);
  fk_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);

  // Resize the variables for 6 DOF
  q_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque setup
  KDL::Vector g(0.0, 0.0, -9.81); // Adjust based on robot orientation
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));

  return true;
}

void ImpedanceControlController::starting(const ros::Time& time) {
  // Initialize joint positions to current values
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    q_(i) = effort_joint_handles_[i].getPosition();
  }
  
  // CRITICAL: Always set the desired position to current position initially
  // This prevents initial jumps and large errors
  KDL::Frame current_frame;
  if (fk_solver_->JntToCart(q_, current_frame) >= 0) {
    // Set current position as target
    x_d_.x(current_frame.p.x());
    x_d_.y(current_frame.p.y());
    x_d_.z(current_frame.p.z());
    
    // Set current orientation as target
    double roll, pitch, yaw;
    current_frame.M.GetRPY(roll, pitch, yaw);
    x_o_.x(roll);
    x_o_.y(pitch);
    x_o_.z(yaw);
    
    ROS_INFO_STREAM("Starting impedance controller at current position: " 
                    << x_d_.x() << ", " << x_d_.y() << ", " << x_d_.z());
    ROS_INFO_STREAM("Current orientation (RPY): " 
                    << roll << ", " << pitch << ", " << yaw);
  } else {
    ROS_ERROR("Failed to get current end-effector pose. Using zero position.");
    x_d_ = KDL::Vector(0.0, 0.0, 0.0);
    x_o_ = KDL::Vector(0.0, 0.0, 0.0);
  }
}

void ImpedanceControlController::update(const ros::Time& time,
                                      const ros::Duration& period)
{
  // Get current joint positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    q_(i) = effort_joint_handles_[i].getPosition();
  }

  // Calculate Jacobian
  KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
  if (jnt_to_jac_solver_->JntToJac(q_, jacobian) >= 0) {
    // Publish Jacobian
    std_msgs::Float64MultiArray jacobian_msg;
    jacobian_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    jacobian_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

    // Set up dimensions for the Jacobian matrix
    jacobian_msg.layout.dim[0].label = "rows";
    jacobian_msg.layout.dim[0].size = jacobian.rows();
    jacobian_msg.layout.dim[0].stride = jacobian.rows() * jacobian.columns();
    jacobian_msg.layout.dim[1].label = "columns";
    jacobian_msg.layout.dim[1].size = jacobian.columns();
    jacobian_msg.layout.dim[1].stride = jacobian.columns();

    // Reserve space for all elements
    jacobian_msg.data.reserve(jacobian.rows() * jacobian.columns());

    // Populate the data
    for (unsigned int i = 0; i < jacobian.rows(); ++i) {
      for (unsigned int j = 0; j < jacobian.columns(); ++j) {
        jacobian_msg.data.push_back(jacobian(i, j));
      }
    }

    // Publish the Jacobian matrix
    jacobian_publisher_.publish(jacobian_msg);
  }

  // Calculate forward kinematics
  KDL::Frame end_effector_frame;
  geometry_msgs::Vector3 euler_angles;
  
  if (fk_solver_->JntToCart(q_, end_effector_frame) >= 0) {
    // Publish end effector position
    geometry_msgs::Point end_effector_pos;
    end_effector_pos.x = end_effector_frame.p.x();
    end_effector_pos.y = end_effector_frame.p.y();
    end_effector_pos.z = end_effector_frame.p.z();
    end_effector_pos_publisher_.publish(end_effector_pos);

    // Calculate and publish orientation as Euler angles
    double roll, pitch, yaw;
    end_effector_frame.M.GetRPY(roll, pitch, yaw);
    euler_angles.x = roll;
    euler_angles.y = pitch;
    euler_angles.z = yaw;
    end_effector_orient_euler_publisher_.publish(euler_angles);
  }

  // Calculate orientation error
  double error_roll = x_o_.x() - euler_angles.x;
  double error_pitch = x_o_.y() - euler_angles.y;
  double error_yaw = x_o_.z() - euler_angles.z;

  // Normalize angles to be within the range of -π to π
  error_roll = fmod(error_roll + M_PI, 2 * M_PI) - M_PI;
  error_pitch = fmod(error_pitch + M_PI, 2 * M_PI) - M_PI;
  error_yaw = fmod(error_yaw + M_PI, 2 * M_PI) - M_PI;

  // Get joint velocities
  std_msgs::Float64MultiArray velocity_msg;
  velocity_msg.data.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    double velocity = effort_joint_handles_[i].getVelocity();
    velocity_msg.data.push_back(velocity);
  }
  // Publish joint velocities
  velocity_publisher_.publish(velocity_msg);

  // Convert joint velocities to Eigen vector for calculation
  Eigen::VectorXd joint_velocities(velocity_msg.data.size());
  for (size_t i = 0; i < velocity_msg.data.size(); ++i) {
    joint_velocities[i] = velocity_msg.data[i];
  }

  // Calculate end-effector velocity in Cartesian space
  Eigen::VectorXd end_effector_velocity = jacobian.data * joint_velocities;

  // Publish end-effector velocity
  std_msgs::Float64MultiArray ee_velocity_msg;
  ee_velocity_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ee_velocity_msg.layout.dim[0].size = end_effector_velocity.size();
  ee_velocity_msg.layout.dim[0].stride = 1;
  ee_velocity_msg.layout.dim[0].label = "End Effector Velocity";

  ee_velocity_msg.data.resize(end_effector_velocity.size());
  for (int i = 0; i < end_effector_velocity.size(); ++i) {
    ee_velocity_msg.data[i] = end_effector_velocity[i];
  }
  end_eff_velocity_publisher_.publish(ee_velocity_msg);

  // Compute the gravity torque - ensure this is calculated correctly
  KDL::JntArray tau_gravity(kdl_chain_.getNrOfJoints());
  int gravity_result = MCG_solver_->JntToGravity(q_, tau_gravity);
  
  if (gravity_result < 0) {
    ROS_ERROR_STREAM("Failed to compute gravity compensation! Error code: " << gravity_result);
    // In case of failure, use zero gravity compensation (safer than using garbage values)
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
      tau_gravity(i) = 0.0;
    }
  }
  
  // Verify gravity compensation values are reasonable
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    if (!std::isfinite(tau_gravity(i)) || std::abs(tau_gravity(i)) > 50.0) {
      ROS_WARN_STREAM_THROTTLE(1.0, "Suspicious gravity compensation value for joint " 
                              << i << ": " << tau_gravity(i) << " - clamping");
      tau_gravity(i) = std::max(-10.0, std::min(tau_gravity(i), 10.0));
    }
  }

  // Calculate position error in Cartesian space
  KDL::Vector error_trans = x_d_ - end_effector_frame.p;

  // Compute the spring force in Cartesian space
  KDL::Vector spring_force_trans(
    error_trans.x() * K_trans_.x(), 
    error_trans.y() * K_trans_.y(), 
    error_trans.z() * K_trans_.z()
  );
  
  KDL::Vector spring_force_rot(
    K_rot_.x() * error_roll, 
    K_rot_.y() * error_pitch, 
    K_rot_.z() * error_yaw
  );
  
  // Convert spring force into joint torques using Jacobian transpose
  Eigen::VectorXd spring_force_vec(6); // 6 DOF: 3 translations + 3 rotations
  
  // First check if Jacobian is valid (avoid singular configurations)
  bool jacobian_valid = true;
  for (unsigned int i = 0; i < jacobian.rows(); ++i) {
    for (unsigned int j = 0; j < jacobian.columns(); ++j) {
      if (!std::isfinite(jacobian(i, j))) {
        jacobian_valid = false;
        ROS_WARN_THROTTLE(1.0, "Invalid Jacobian detected! Using gravity compensation only.");
        break;
      }
    }
    if (!jacobian_valid) break;
  }
  
  if (!jacobian_valid) {
    // Set spring and damping forces to zero, rely only on gravity comp
    spring_force_vec.setZero();
    Eigen::VectorXd tau_spring_vec = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
    Eigen::VectorXd tau_damping_vec = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
    return; // Exit early and only apply gravity compensation
  }
  
  Eigen::MatrixXd jacobian_transpose = jacobian.data.transpose();
  
  // Limit the maximum positional error to prevent large forces
  double max_pos_error = 0.05; // 5cm - even more conservative
  double max_rot_error = 0.2;  // ~11 degrees - more conservative
  
  // Clamp translational error
  KDL::Vector clamped_trans_error(
    std::max(-max_pos_error, std::min(error_trans.x(), max_pos_error)),
    std::max(-max_pos_error, std::min(error_trans.y(), max_pos_error)),
    std::max(-max_pos_error, std::min(error_trans.z(), max_pos_error))
  );
  
  // Clamp rotational error
  double clamped_roll = std::max(-max_rot_error, std::min(error_roll, max_rot_error));
  double clamped_pitch = std::max(-max_rot_error, std::min(error_pitch, max_rot_error));
  double clamped_yaw = std::max(-max_rot_error, std::min(error_yaw, max_rot_error));
  
  // Recalculate forces with clamped errors
  KDL::Vector clamped_spring_force_trans(
    clamped_trans_error.x() * K_trans_.x(), 
    clamped_trans_error.y() * K_trans_.y(), 
    clamped_trans_error.z() * K_trans_.z()
  );
  
  KDL::Vector clamped_spring_force_rot(
    K_rot_.x() * clamped_roll, 
    K_rot_.y() * clamped_pitch, 
    K_rot_.z() * clamped_yaw
  );
  
  spring_force_vec << 
    clamped_spring_force_trans.x(), clamped_spring_force_trans.y(), clamped_spring_force_trans.z(),
    clamped_spring_force_rot.x(), clamped_spring_force_rot.y(), clamped_spring_force_rot.z();

  Eigen::VectorXd tau_spring_vec = jacobian_transpose * spring_force_vec;
  
  // Log the forces for debugging
  ROS_DEBUG_STREAM("Position error: [" << error_trans.x() << ", " << error_trans.y() << ", " << error_trans.z() << "]");
  ROS_DEBUG_STREAM("Spring force: [" << clamped_spring_force_trans.x() << ", " 
                                     << clamped_spring_force_trans.y() << ", " 
                                     << clamped_spring_force_trans.z() << "]");

  // Compute damping (e_dot is the negative of end-effector velocity)
  Eigen::VectorXd e_dot(6);
  e_dot << -end_effector_velocity.head(3), -end_effector_velocity.tail(3);

  // Apply damping in Cartesian space - ensure stability with proper damping
  Eigen::VectorXd damping_force(6);
  
  // Calculate a scaling factor for damping based on velocity magnitude
  // This adds extra damping when velocity is high (critical for stability)
  double vel_trans_magnitude = e_dot.head(3).norm();
  double vel_rot_magnitude = e_dot.tail(3).norm();
  
  // Increase damping dynamically if velocity gets too high
  double trans_scale = 1.0 + 2.0 * std::max(0.0, vel_trans_magnitude - 0.05);
  double rot_scale = 1.0 + 2.0 * std::max(0.0, vel_rot_magnitude - 0.1);
  
  // Apply scaled damping forces
  damping_force << 
    trans_scale * Kd_lin_.x() * e_dot[0], 
    trans_scale * Kd_lin_.y() * e_dot[1], 
    trans_scale * Kd_lin_.z() * e_dot[2],
    rot_scale * Kd_ang_.x() * e_dot[3], 
    rot_scale * Kd_ang_.y() * e_dot[4], 
    rot_scale * Kd_ang_.z() * e_dot[5];

  Eigen::VectorXd tau_damping_vec = jacobian_transpose * damping_force;
  
  // Log damping information for debugging
  ROS_DEBUG_STREAM("Velocity magnitude: trans=" << vel_trans_magnitude 
                  << " rot=" << vel_rot_magnitude);
  ROS_DEBUG_STREAM("Damping scale: trans=" << trans_scale << " rot=" << rot_scale);

  // Final Impedance control equation: tau = G + J^T * (K*e + D*e_dot)
  KDL::JntArray tau_total(kdl_chain_.getNrOfJoints());
  
  // First apply only gravity compensation for all joints
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    tau_total(i) = tau_gravity(i);
  }
  
  // Then carefully add impedance control terms for stability
  for (int i = 0; i < kdl_chain_.getNrOfJoints() && i < tau_spring_vec.size(); i++) {
    // Ensure we don't exceed array bounds
    double spring_term = (i < tau_spring_vec.size()) ? tau_spring_vec(i) : 0.0;
    double damping_term = (i < tau_damping_vec.size()) ? tau_damping_vec(i) : 0.0;
    
    // Apply very strict torque limiting for each component separately
    double max_stiffness_torque = 10.0; // Strict limit for stiffness torque
    double max_damping_torque = 10.0;   // Higher limit for damping (stabilizing)
    
    // Clamp individual components
    spring_term = std::max(-max_stiffness_torque, std::min(spring_term, max_stiffness_torque));
    damping_term = std::max(-max_damping_torque, std::min(damping_term, max_damping_torque));
    
    // Add clamped terms to gravity compensation
    tau_total(i) += spring_term + damping_term;
    
    // Final safety clamp on total torque
    double max_total_torque = 10; // Maximum total torque allowed
    tau_total(i) = std::max(-max_total_torque, std::min(tau_total(i), max_total_torque));
    
    // Debug output - use ROS_INFO for initial debugging, change to DEBUG later
    ROS_DEBUG_STREAM("Joint " << i << ": G=" << tau_gravity(i) 
                  << " Spring=" << spring_term 
                  << " Damping=" << damping_term 
                  << " Total=" << tau_total(i));
  }

  // Publish torques
  std_msgs::Float64MultiArray torque_msg;
  torque_msg.data.clear();
  for (size_t i = 0; i < tau_total.rows(); i++) {
    torque_msg.data.push_back(tau_total(i));
  }
  torque_publisher_.publish(torque_msg);

  // Command torques to hardware - similar to gravity compensation's approach
  for (size_t i = 0; i < tau_total.rows(); i++) {
    // Only send commands if values are valid (not NaN or inf)
    if (std::isfinite(tau_total(i))) {
      effort_joint_handles_[i].setCommand(tau_total(i));
      
      // Print similar debug info as in gravity compensation controller
      if (i == 0) {  // Only print for first joint to reduce console spam
        ROS_INFO_STREAM_THROTTLE(1.0, "Sending torque " << i << " " << tau_total(i));
      }
    } else {
      // If we get invalid torque, fall back to gravity compensation only
      if (std::isfinite(tau_gravity(i))) {
        effort_joint_handles_[i].setCommand(tau_gravity(i));
        ROS_WARN_STREAM_THROTTLE(1.0, "Invalid impedance torque, using gravity comp for joint " << i);
      } else {
        effort_joint_handles_[i].setCommand(0.0);
        ROS_ERROR_STREAM_THROTTLE(1.0, "Invalid gravity torque for joint " << i << ", setting zero torque");
      }
    }
  }
}

// Callback to update desired pose from external source
void ImpedanceControlController::desiredPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  // Update desired position
  x_d_.x(msg->position.x);
  x_d_.y(msg->position.y);
  x_d_.z(msg->position.z);
  
  // Convert quaternion to RPY (Roll-Pitch-Yaw)
  double roll, pitch, yaw;
  KDL::Rotation rot = KDL::Rotation::Quaternion(
    msg->orientation.x, 
    msg->orientation.y, 
    msg->orientation.z, 
    msg->orientation.w
  );
  rot.GetRPY(roll, pitch, yaw);
  
  // Update desired orientation
  x_o_.x(roll);
  x_o_.y(pitch);
  x_o_.z(yaw);
}

void ImpedanceControlController::stopping(const ros::Time& time) {
  // Clean shutdown behavior - zero out commands
  for (size_t i = 0; i < effort_joint_handles_.size(); i++) {
    effort_joint_handles_[i].setCommand(0.0);
  }
}

}  // namespace advanced_robotics_control

PLUGINLIB_EXPORT_CLASS(
    advanced_robotics_control::ImpedanceControlController,
    controller_interface::ControllerBase)