#include "s_curve_planner/s_curve_controller.hpp"

#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "s_curve_planner/left_arm_ik.hpp"
#include "s_curve_planner/right_arm_ik.hpp"

namespace s_curve_planner {

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_ACCELERATION;

SCurveTrajectoryController::SCurveTrajectoryController() {}

controller_interface::InterfaceConfiguration
SCurveTrajectoryController::command_interface_configuration() const {
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
SCurveTrajectoryController::state_interface_configuration() const {
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

controller_interface::CallbackReturn SCurveTrajectoryController::on_init() {
  left_names_ = {"l_post_Joint", "l_shoulder_Joint", "l_arm_JointA", "l_arm_JointB",
                 "l_arm_JointC", "l_wrist_p_Joint", "l_wrist_y_Joint", "l_wrist_r_Joint"};
  right_names_ = {"r_post_Joint", "r_shoulder_Joint", "r_arm_JointA", "r_arm_JointB",
                  "r_arm_JointC", "r_wrist_p_Joint", "r_wrist_y_Joint", "r_wrist_r_Joint"};
  left_q_init_ = {0.5, 0.5, 0.04, 0.04, 0.04, 0.0, 0.0, 0.0};
  right_q_init_ = {0.5, -0.5, 0.04, 0.04, 0.04, 0.0, 0.0, 0.0};

  joint_params_.resize(TOTAL_JOINTS);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    int idx = static_cast<int>(i % ARM_JOINTS);
    bool pris = (idx == 0 || idx == 2 || idx == 3 || idx == 4);
    joint_params_[i].v_max = pris ? 0.12 : 1.0;
    joint_params_[i].a_max = pris ? 1.5  : 3.0;
    joint_params_[i].j_max = pris ? 15.0 : 25.0;
  }

  start_pos_.resize(TOTAL_JOINTS, 0.0);
  target_pos_.resize(TOTAL_JOINTS, 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SCurveTrajectoryController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  left_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/left_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { left_callback(m); });
  right_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/right_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { right_callback(m); });
  dual_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dual_tcp", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { dual_callback(m); });

  RCLCPP_INFO(node->get_logger(), "SCurveTrajectoryController configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SCurveTrajectoryController::on_activate(const rclcpp_lifecycle::State&) {
  plan_ready_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SCurveTrajectoryController::on_deactivate(const rclcpp_lifecycle::State&) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void SCurveTrajectoryController::left_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 6) return;

  KDL::Vector pos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation rot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  auto res = left_arm_ik::solve_ik(pos, rot, left_q_init_);
  if (!res.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Left IK failed pos_err=%.4f rot_err=%.4f",
                res.pos_err, res.rot_err);
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  left_q_init_ = res.q8;

  std::vector<double> p0(TOTAL_JOINTS, 0.0), p1(TOTAL_JOINTS, 0.0);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    p0[i] = state_interfaces_[i * 3].get_value();
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    p1[i] = res.q8[i];
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    p1[ARM_JOINTS + i] = p0[ARM_JOINTS + i];
  }

  synced_plan(p0, p1);
}

void SCurveTrajectoryController::right_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 6) return;

  KDL::Vector pos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation rot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  auto res = right_arm_ik::solve_ik(pos, rot, right_q_init_);
  if (!res.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Right IK failed pos_err=%.4f rot_err=%.4f",
                res.pos_err, res.rot_err);
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  right_q_init_ = res.q8;

  std::vector<double> p0(TOTAL_JOINTS, 0.0), p1(TOTAL_JOINTS, 0.0);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    p0[i] = state_interfaces_[i * 3].get_value();
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    p1[i] = p0[i];
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    p1[ARM_JOINTS + i] = res.q8[i];
  }

  synced_plan(p0, p1);
}

void SCurveTrajectoryController::dual_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
  if (msg->data.size() < 12) return;

  KDL::Vector lpos(msg->data[0], msg->data[1], msg->data[2]);
  KDL::Rotation lrot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
  KDL::Vector rpos(msg->data[6], msg->data[7], msg->data[8]);
  KDL::Rotation rrot = KDL::Rotation::RPY(msg->data[11], msg->data[10], msg->data[9]);

  auto lres = left_arm_ik::solve_ik(lpos, lrot, left_q_init_);
  auto rres = right_arm_ik::solve_ik(rpos, rrot, right_q_init_);

  if (!lres.success || !rres.success) {
    RCLCPP_WARN(get_node()->get_logger(), "Dual IK: L=%d R=%d", lres.success, rres.success);
    return;
  }

  std::lock_guard<std::mutex> l(mtx_);
  left_q_init_ = lres.q8;
  right_q_init_ = rres.q8;

  std::vector<double> p0(TOTAL_JOINTS, 0.0), p1(TOTAL_JOINTS, 0.0);
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    p0[i] = state_interfaces_[i * 3].get_value();
  }
  for (size_t i = 0; i < ARM_JOINTS; i++) {
    p1[i] = lres.q8[i];
    p1[ARM_JOINTS + i] = rres.q8[i];
  }

  synced_plan(p0, p1);
}

void SCurveTrajectoryController::synced_plan(const std::vector<double>& p0,
                                                const std::vector<double>& p1) {
  if (!sync_traj_.plan(p0, p1, joint_params_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Synced trajectory planning failed");
    return;
  }
  start_pos_ = p0;
  target_pos_ = p1;
  elapsed_time_ = 0.0;
  plan_ready_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Plan ready: sync_time=%.3fs", sync_traj_.sync_time());
}

double SCurveTrajectoryController::compute_progress() const {
  double min_progress = 1.0;
  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    double disp = target_pos_[i] - start_pos_[i];
    if (std::fabs(disp) < 1e-12) continue;
    double actual = state_interfaces_[i * 3].get_value();
    double prog = (actual - start_pos_[i]) / disp;
    prog = std::max(0.0, std::min(1.0, prog));
    if (prog < min_progress) min_progress = prog;
  }
  return min_progress;
}

controller_interface::return_type
SCurveTrajectoryController::update(const rclcpp::Time&, const rclcpp::Duration& period) {
  std::lock_guard<std::mutex> l(mtx_);

  if (!plan_ready_) {
    for (size_t i = 0; i < TOTAL_JOINTS; i++) {
      double pos = state_interfaces_[i * 3].get_value();
      command_interfaces_[i * 3].set_value(pos);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  elapsed_time_ += period.seconds();

  double sync_T = sync_traj_.sync_time();
  double s = compute_progress();

  if (s >= 1.0 || elapsed_time_ >= sync_T) {
    for (size_t i = 0; i < TOTAL_JOINTS; i++) {
      command_interfaces_[i * 3].set_value(target_pos_[i]);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
    }
    plan_ready_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
                "Trajectory complete: progress=%.3f time=%.3f/%.3f", s, elapsed_time_, sync_T);
    return controller_interface::return_type::OK;
  }

  auto feedforward = sync_traj_.evaluate(elapsed_time_);

  for (size_t i = 0; i < TOTAL_JOINTS; i++) {
    double actual = state_interfaces_[i * 3].get_value();
    double target = target_pos_[i];
    double remaining = std::fabs(target - actual);
    double dead_zone = is_prismatic(i) ? DEAD_ZONE_PRIS : DEAD_ZONE_REV;
    double v_min = is_prismatic(i) ? V_MIN_PRIS : V_MIN_REV;

    if (remaining <= dead_zone) {
      command_interfaces_[i * 3].set_value(target);
      command_interfaces_[i * 3 + 1].set_value(0.0);
      command_interfaces_[i * 3 + 2].set_value(0.0);
      continue;
    }

    double planned_pos = feedforward[i].position;
    double planned_vel = feedforward[i].velocity;
    double error = planned_pos - actual;
    double cmd_vel = planned_vel + KP * error;

    if (std::fabs(cmd_vel) < v_min) {
      cmd_vel = std::copysign(v_min, cmd_vel);
    }

    command_interfaces_[i * 3].set_value(planned_pos);
    command_interfaces_[i * 3 + 1].set_value(cmd_vel);
    command_interfaces_[i * 3 + 2].set_value(0.0);
  }

  return controller_interface::return_type::OK;
}

}  // namespace s_curve_planner

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(s_curve_planner::SCurveTrajectoryController,
                       controller_interface::ControllerInterface)
