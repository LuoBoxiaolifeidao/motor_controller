#include "mppi_controller/mppi_controller.hpp"

#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "mppi_controller/left_arm_ik.hpp"
#include "mppi_controller/right_arm_ik.hpp"

#include "mppi_controller/collision_check.hpp"

namespace mppi_controller {

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_ACCELERATION;

MPPIController::MPPIController() {}

controller_interface::InterfaceConfiguration
MPPIController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    conf.names.push_back(left_names_[i] + "/" + HW_IF_POSITION);
    conf.names.push_back(left_names_[i] + "/" + HW_IF_VELOCITY);
    conf.names.push_back(left_names_[i] + "/" + HW_IF_ACCELERATION);
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    conf.names.push_back(right_names_[i] + "/" + HW_IF_POSITION);
    conf.names.push_back(right_names_[i] + "/" + HW_IF_VELOCITY);
    conf.names.push_back(right_names_[i] + "/" + HW_IF_ACCELERATION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MPPIController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    conf.names.push_back(left_names_[i] + "/" + HW_IF_POSITION);
    conf.names.push_back(left_names_[i] + "/" + HW_IF_VELOCITY);
    conf.names.push_back(left_names_[i] + "/" + HW_IF_ACCELERATION);
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    conf.names.push_back(right_names_[i] + "/" + HW_IF_POSITION);
    conf.names.push_back(right_names_[i] + "/" + HW_IF_VELOCITY);
    conf.names.push_back(right_names_[i] + "/" + HW_IF_ACCELERATION);
  }
  return conf;
}

controller_interface::CallbackReturn MPPIController::on_init() {
  left_names_ = {"l_post_Joint", "l_shoulder_Joint", "l_arm_JointA", "l_arm_JointB",
                 "l_arm_JointC", "l_wrist_p_Joint", "l_wrist_y_Joint", "l_wrist_r_Joint"};
  right_names_ = {"r_post_Joint", "r_shoulder_Joint", "r_arm_JointA", "r_arm_JointB",
                  "r_arm_JointC", "r_wrist_p_Joint", "r_wrist_y_Joint", "r_wrist_r_Joint"};
  left_q_init_ = {0.5, 0.5, 0.04, 0.04, 0.04, 0.0, 0.0, 0.0};
  right_q_init_ = {0.5, -0.5, 0.04, 0.04, 0.04, 0.0, 0.0, 0.0};

  q_min_.resize(TOTAL_JOINTS); q_max_.resize(TOTAL_JOINTS);
  for (int i = 0; i < 8; i++) {
    q_min_[i]     = left_arm_ik::JOINT_LOWER[i];
    q_max_[i]     = left_arm_ik::JOINT_UPPER[i];
    q_min_[8 + i] = right_arm_ik::JOINT_LOWER[i];
    q_max_[8 + i] = right_arm_ik::JOINT_UPPER[i];
  }

  u_min_.resize(TOTAL_JOINTS); u_max_.resize(TOTAL_JOINTS);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) { 
    double v = is_prismatic(i) ? 0.10 : 0.20;
    u_min_[i] = -v; u_max_[i] = v;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MPPIController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  mppi_cfg_.horizon = 6;
  mppi_cfg_.dt = 0.05;
  mppi_cfg_.num_samples = 128;
  mppi_cfg_.lambda = 120.0;
  mppi_cfg_.sigma_pris = 0.020;
  mppi_cfg_.sigma_rev = 0.080;
  mppi_cfg_.goal_weight = 10.0;
  mppi_cfg_.control_weight = 0.05;
  mppi_cfg_.collision_penalty = 2000.0;
  mppi_cfg_.terminal_vel_weight = 10.0;
  mppi_cfg_.collision_check_stride = 2;
  mppi_cfg_.arm_count = 0;
  mppi_cfg_.joints_per_arm = 8;
  mppi_cfg_.priority_eps = 0.01;

  auto is_pris = [](size_t j) -> bool { return is_prismatic(j); };
  mppi_ = std::make_unique<MPPIPlanner>(mppi_cfg_, TOTAL_JOINTS, is_pris);

  left_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/left_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { left_callback(m); });
  right_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/right_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { right_callback(m); });
  dual_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dual_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { dual_callback(m); });

  RCLCPP_INFO(node->get_logger(), "MPPIController configured, FCL ready");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MPPIController::on_activate(const rclcpp_lifecycle::State&) {
  std::lock_guard<std::mutex> l(mtx_);
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    left_q_target_[i] = state_interfaces_[i * 3].get_value();
    right_q_target_[i] = state_interfaces_[(ARM_JOINTS + i) * 3].get_value();
  }
  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MPPIController::on_deactivate(const rclcpp_lifecycle::State&) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void MPPIController::left_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 6) return;

  KDL::Vector pos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation rot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  auto res = left_arm_ik::solve_ik(pos, rot, left_q_init_);
  if (!res.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Left IK failed");
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  left_q_init_ = res.q8;
  for (size_t i = 0; i < ARM_JOINTS; i++) left_q_target_[i] = res.q8[i];
  left_has_target_ = true;
  active_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Left target: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
    left_q_target_[0], left_q_target_[1], left_q_target_[2], left_q_target_[3],
    left_q_target_[4], left_q_target_[5], left_q_target_[6], left_q_target_[7]);
}

void MPPIController::right_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 6) return;

  KDL::Vector pos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation rot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  auto res = right_arm_ik::solve_ik(pos, rot, right_q_init_);
  if (!res.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Right IK failed");
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  right_q_init_ = res.q8;
  for (size_t i = 0; i < ARM_JOINTS; i++) right_q_target_[i] = res.q8[i];
  right_has_target_ = true;
  active_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Right target: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
    right_q_target_[0], right_q_target_[1], right_q_target_[2], right_q_target_[3],
    right_q_target_[4], right_q_target_[5], right_q_target_[6], right_q_target_[7]);
}

void MPPIController::dual_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 12) return;

  KDL::Vector lpos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation lrot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  KDL::Vector rpos(msg->data[6], msg->data[7], msg->data[8]);
  KDL::Rotation rrot = KDL::Rotation::RPY(msg->data[11], msg->data[10], msg->data[9]);

  auto lres = left_arm_ik::solve_ik(lpos, lrot, left_q_init_);
  auto rres = right_arm_ik::solve_ik(rpos, rrot, right_q_init_);
  if (!lres.success || !rres.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Dual IK failed");
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  left_q_init_ = lres.q8;
  right_q_init_ = rres.q8;
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    left_q_target_[i] = lres.q8[i];
    right_q_target_[i] = rres.q8[i];
  }
  left_has_target_ = true;
  right_has_target_ = true;
  active_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Dual target set");
}

controller_interface::return_type
MPPIController::update(const rclcpp::Time&, const rclcpp::Duration& period) {
  double dt = period.seconds();
  std::lock_guard<std::mutex> l(mtx_);

  std::vector<double> q_curr(TOTAL_JOINTS);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    q_curr[i] = state_interfaces_[i * 3].get_value();
  }

  if (!active_) {
    for (size_t i = 0; i < TOTAL_JOINTS; i++) {
      command_interfaces_[i * 3].set_value(q_curr[i]);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  std::vector<double> q_goal(TOTAL_JOINTS);
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    q_goal[i] = left_q_target_[i];
    q_goal[ARM_JOINTS + i] = right_q_target_[i];
  }

  bool all_done = true;
  static int done_cycles = 0;
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    double dz = is_prismatic(i) ? 0.005 : 0.02;
    if (std::fabs(q_goal[i] - q_curr[i]) > dz) { all_done = false; break; }
  }
  if (all_done) done_cycles++; else done_cycles = 0;
  if (done_cycles > 5) {
    RCLCPP_INFO(get_node()->get_logger(), "MPPI: target reached, stopping");
    for (size_t i = 0; i < TOTAL_JOINTS; i++) {
      command_interfaces_[i * 3].set_value(q_goal[i]);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    }
    active_ = false;
    left_has_target_ = false;
    right_has_target_ = false;
    done_cycles = 0;
    return controller_interface::return_type::OK;
  }

  auto check_coll = [](const std::vector<double>& q) -> bool {
    std::string wp; double wd;
    return mppi_controller::check_collision(q, &wp, &wd);
  };

  int coll_hits = 0;
  auto u = mppi_->plan_step(q_curr, q_goal, q_min_, q_max_,
                            u_min_, u_max_, check_coll, coll_hits);

  static int log_cycle = 0;
  log_cycle++;
  if (log_cycle % 50 == 0) {
    double du = 0.0;
    for (auto& v : u) du += v * v;
    du = std::sqrt(du);

    double err_l = 0.0, err_r = 0.0;
    for (size_t i = 0; i < ARM_JOINTS; i++) {
      double e = q_curr[i] - q_goal[i];
      err_l += e * e;
      double er = q_curr[ARM_JOINTS + i] - q_goal[ARM_JOINTS + i];
      err_r += er * er;
    }
    err_l = std::sqrt(err_l); err_r = std::sqrt(err_r);

    std::string wp; double wd;
    check_collision(q_curr, &wp, &wd);
    std::string fw_pair; double fw_d = mppi_controller::finger_wrist_dist(q_curr, &fw_pair);

    RCLCPP_INFO(get_node()->get_logger(),
      "MPPI: coll=%d/128 |u|=%.4f errL=%.3f errR=%.3f  "
      "dist=%.3f %s  fw=%.3f %s  "
      "cL=[%.2f %.2f %.2f ..] gL=[%.2f %.2f %.2f ..] "
      "cR=[%.2f %.2f %.2f ..] gR=[%.2f %.2f %.2f ..]",
      coll_hits, du, err_l, err_r,
      wd, wp.c_str(), fw_d, fw_pair.c_str(),
      q_curr[0], q_curr[1], q_curr[2], q_goal[0], q_goal[1], q_goal[2],
      q_curr[8], q_curr[9], q_curr[10], q_goal[8], q_goal[9], q_goal[10]);
  }

  bool lock_left = (right_has_target_ && !left_has_target_);
  bool lock_right = (left_has_target_ && !right_has_target_);

  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    bool locked = (i < ARM_JOINTS) ? lock_left : lock_right;
    if (locked) {
      command_interfaces_[i * 3].set_value(q_goal[i]);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    } else {
      double cmd_pos = q_curr[i] + u[i] * dt;
      cmd_pos = std::clamp(cmd_pos, q_min_[i], q_max_[i]);
      command_interfaces_[i * 3].set_value(cmd_pos);
      command_interfaces_[i * 3 + 1].set_value(u[i]);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace mppi_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mppi_controller::MPPIController,
                       controller_interface::ControllerInterface)
