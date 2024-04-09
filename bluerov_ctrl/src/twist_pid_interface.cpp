// Copyright (C) 2023  Niklas Trekel

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "bluerov_ctrl/twist_pid_interface.hpp"

namespace bluerov_ctrl {
void TwistPIDInterface::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  accelerations_sub_ =
      node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
          "accelerations", rclcpp::SystemDefaultsQoS(),
          std::bind(&TwistPIDInterface::onAccelerations, this,
                    std::placeholders::_1));
  initializeController();
  acceleration_msg_.twist.linear.x = 0.0;
  acceleration_msg_.twist.linear.y = 0.0;
  acceleration_msg_.twist.linear.z = 0.0;
  acceleration_msg_.twist.angular.x = 0.0;
  acceleration_msg_.twist.angular.y = 0.0;
  acceleration_msg_.twist.angular.z = 0.0;
  // initialize debug publishers
  debug_thrust_p_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/thrust_p_component", rclcpp::SystemDefaultsQoS());
  debug_thrust_i_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/thrust_i_component", rclcpp::SystemDefaultsQoS());
  debug_thrust_d_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/thrust_d_component", rclcpp::SystemDefaultsQoS());
  debug_torque_p_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/torque_p_component", rclcpp::SystemDefaultsQoS());
  debug_torque_i_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/torque_i_component", rclcpp::SystemDefaultsQoS());
  debug_torque_d_pub_ =
      node_ptr_->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
          "~/torque_d_component", rclcpp::SystemDefaultsQoS());

  accel_timeout_timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), std::chrono::milliseconds(500),
      std::bind(&TwistPIDInterface::onAccelTimeout, this));
}

void TwistPIDInterface::initializeController() {
  // load parameters
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  bool body_fixed = true;
  name = "twist_pid.body_fixed";
  descr_text =
      "Decide if controller acts in body or earth frame (integral term)";
  descr = hippo_common::param_utils::Description(descr_text);
  body_fixed = node_ptr_->declare_parameter(name, body_fixed, descr);

  controller_ = new TwistPIDControl(body_fixed);
}

void TwistPIDInterface::update(
    const double &dt, const nav_msgs::msg::Odometry &msg,
    hippo_control_msgs::msg::ActuatorSetpoint &out_thrust,
    hippo_control_msgs::msg::ActuatorSetpoint &out_torque) {
  Eigen::Quaterniond att_eigen;
  Eigen::Vector3d vel_eigen, acc_eigen, ang_vel_eigen, ang_acc_eigen;
  Vector6d out_controls;
  hippo_common::convert::RosToEigen(msg.pose.pose.orientation, att_eigen);
  hippo_common::convert::RosToEigen(msg.twist.twist.linear, vel_eigen);
  hippo_common::convert::RosToEigen(msg.twist.twist.angular, ang_vel_eigen);
  hippo_common::convert::RosToEigen(acceleration_msg_.twist.linear, acc_eigen);
  hippo_common::convert::RosToEigen(acceleration_msg_.twist.angular,
                                    ang_acc_eigen);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_->update(dt, att_eigen, vel_eigen, acc_eigen, ang_vel_eigen,
                        ang_acc_eigen, out_controls);
  }
  out_thrust.x = out_controls(0);
  out_thrust.y = out_controls(1);
  out_thrust.z = out_controls(2);
  out_torque.x = out_controls(3);
  out_torque.y = out_controls(4);
  out_torque.z = out_controls(5);
}

void TwistPIDInterface::setVelocityTarget(
    const geometry_msgs::msg::Twist &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  controller_->setVelocityTarget(msg.linear.x, msg.linear.y, msg.linear.z,
                                 msg.angular.x, msg.angular.y, msg.angular.z);
}

void TwistPIDInterface::setAccelerationTarget(
    const geometry_msgs::msg::Twist &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  controller_->setAccelerationTarget(msg.linear.x, msg.linear.y, msg.linear.z,
                                     msg.angular.x, msg.angular.y,
                                     msg.angular.z);
}

void TwistPIDInterface::declareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> axes = {"pos_x", "pos_y", "pos_z",
                                   "att_x", "att_y", "att_z"};
  for (int i = 0; i < int(axes.size()); i++) {
    name = "twist_pid.gain.p." + axes[i];
    descr_text = "Proportional gain for " + axes[i];
    descr = hippo_common::param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_p_(i) = 1.0;
      gain_p_(i) = node_ptr_->declare_parameter(name, gain_p_(i), descr);
      controller_->setPgain(gain_p_(i), i);
    }
  }

  for (int i = 0; i < int(axes.size()); i++) {
    name = "twist_pid.gain.i." + axes[i];
    descr_text = "Integral gain for " + axes[i];
    descr = hippo_common::param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_i_(i) = 0.0;
      gain_i_(i) = node_ptr_->declare_parameter(name, gain_i_(i), descr);
      controller_->setIgain(gain_i_(i), i);
    }
  }

  for (int i = 0; i < int(axes.size()); i++) {
    name = "twist_pid.gain.d." + axes[i];
    descr_text = "Integral gain for " + axes[i];
    descr = hippo_common::param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_d_(i) = 0.0;
      gain_d_(i) = node_ptr_->declare_parameter(name, gain_d_(i), descr);
      controller_->setDgain(gain_d_(i), i);
    }
  }

  name = "twist_pid.d_use_accel_feedforward";
  descr_text = "Decides if acceleration feedforward is used";
  descr = hippo_common::param_utils::Description(descr_text);
  bool d_use_accel_feedforward;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    d_use_accel_feedforward = false;
    d_use_accel_feedforward =
        node_ptr_->declare_parameter(name, d_use_accel_feedforward, descr);
    controller_->setDUseAccelFeedforward(d_use_accel_feedforward);
  }

  name = "twist_pid.d_use_accel_estimation";
  descr_text = "Decides if acceleration feedforward is used";
  descr = hippo_common::param_utils::Description(descr_text);
  bool d_use_accel_estimation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    d_use_accel_estimation = false;
    d_use_accel_estimation =
        node_ptr_->declare_parameter(name, d_use_accel_estimation, descr);
    controller_->setDUseAccelEstimation(d_use_accel_estimation);
  }
}

void TwistPIDInterface::initializeParamCallbacks() {
  p_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&TwistPIDInterface::onSetPgains, this, std::placeholders::_1));
  i_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&TwistPIDInterface::onSetIgains, this, std::placeholders::_1));
  d_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&TwistPIDInterface::onSetDgains, this, std::placeholders::_1));
  d_use_accel_feedforward_cb_handle_ =
      node_ptr_->add_on_set_parameters_callback(
          std::bind(&TwistPIDInterface::onDUseAccelFeedforward, this,
                    std::placeholders::_1));
  d_use_accel_estimation_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&TwistPIDInterface::onDUseAccelEstimation, this,
                std::placeholders::_1));
}

void TwistPIDInterface::onAccelerations(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  accel_timeout_timer_->reset();
  if (accel_timed_out_) {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Received acceleration estimation. Not timed out anymore.");
    accel_timed_out_ = false;
  }
  acceleration_msg_ = *msg;
}

void TwistPIDInterface::onAccelTimeout() {
  if (accel_timed_out_) {
    return;
  }
  RCLCPP_WARN(node_ptr_->get_logger(),
              "Acceleration estimation timed out. Sending zero commands.");
  onTimeout();
  accel_timed_out_ = true;
}

void TwistPIDInterface::publishDebugMsgs() {
  Vector6d component;
  component = controller_->getPComponent();
  publishDebugActuator(debug_thrust_p_pub_, debug_torque_p_pub_, component);
  component = controller_->getIComponent();
  publishDebugActuator(debug_thrust_i_pub_, debug_torque_i_pub_, component);
  component = controller_->getDComponent();
  publishDebugActuator(debug_thrust_d_pub_, debug_torque_d_pub_, component);
}

void TwistPIDInterface::publishDebugActuator(
    const rclcpp::Publisher<
        hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr &thrust_pub,
    const rclcpp::Publisher<
        hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr &torque_pub,
    const Vector6d &data) {
  hippo_control_msgs::msg::ActuatorSetpoint setpoint;
  setpoint.x = data(0);
  setpoint.y = data(1);
  setpoint.z = data(2);
  thrust_pub->publish(setpoint);
  setpoint.x = data(3);
  setpoint.y = data(4);
  setpoint.z = data(5);
  torque_pub->publish(setpoint);
}

rcl_interfaces::msg::SetParametersResult
TwistPIDInterface::onDUseAccelFeedforward(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool d_use_accel_feedforward = false;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "twist_pid.d_use_accel_feedforward",
            d_use_accel_feedforward)) {
      controller_->setDUseAccelFeedforward(d_use_accel_feedforward);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(d_use_accel_feedforward))
                      .c_str());
      break;
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult
TwistPIDInterface::onDUseAccelEstimation(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool d_use_accel_estimation = false;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "twist_pid.d_use_accel_estimation",
            d_use_accel_estimation)) {
      controller_->setDUseAccelEstimation(d_use_accel_estimation);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(d_use_accel_estimation))
                      .c_str());
      break;
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult TwistPIDInterface::onSetPgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> axes = {"pos_x", "pos_y", "pos_z",
                                     "att_x", "att_y", "att_z"};
    for (int i = 0; i < int(axes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "twist_pid.gain.p." + axes[i], gain_p_(i))) {
        controller_->setPgain(gain_p_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gain_p_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult TwistPIDInterface::onSetIgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> axes = {"pos_x", "pos_y", "pos_z",
                                     "att_x", "att_y", "att_z"};
    for (int i = 0; i < int(axes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "twist_pid.gain.i." + axes[i], gain_i_(i))) {
        controller_->setIgain(gain_i_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gain_i_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult TwistPIDInterface::onSetDgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> axes = {"pos_x", "pos_y", "pos_z",
                                     "att_x", "att_y", "att_z"};
    for (int i = 0; i < int(axes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "twist_pid.gain.d." + axes[i], gain_d_(i))) {
        controller_->setDgain(gain_d_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gain_d_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void TwistPIDInterface::onTimeout() { controller_->resetIntegral(); }

}  // namespace bluerov_ctrl
