#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/subscription.hpp>

#include "s_curve_planner/s_curve_trajectory.hpp"
#include "s_curve_planner/synced_trajectory.hpp"

namespace s_curve_planner {

class SCurveTrajectoryController : public controller_interface::ControllerInterface {
public:
  SCurveTrajectoryController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void left_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
  void right_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
  void dual_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

  void synced_plan(const std::vector<double>& p0, const std::vector<double>& p1);

  double compute_progress() const;

  static bool is_prismatic(size_t joint_idx) {
    int idx = static_cast<int>(joint_idx % ARM_JOINTS);
    return idx == 0 || idx == 2 || idx == 3 || idx == 4;
  }

  static constexpr size_t ARM_JOINTS = 8;
  static constexpr size_t TOTAL_JOINTS = 16;

  static constexpr double DEAD_ZONE_REV = 0.008726;   // 0.5 deg in rad
  static constexpr double DEAD_ZONE_PRIS = 0.001;     // 1 mm
  static constexpr double V_MIN_REV = 0.01;           // rad/s
  static constexpr double V_MIN_PRIS = 0.001;         // m/s
  static constexpr double KP = 15.0;

  std::array<std::string, ARM_JOINTS> left_names_;
  std::array<std::string, ARM_JOINTS> right_names_;

  std::array<double, ARM_JOINTS> left_q_init_;
  std::array<double, ARM_JOINTS> right_q_init_;

  std::mutex mtx_;
  SyncedTrajectory sync_traj_;
  std::vector<SCurveParams> joint_params_;
  std::vector<double> start_pos_;
  std::vector<double> target_pos_;
  double elapsed_time_ = 0.0;
  bool plan_ready_ = false;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dual_sub_;
};

}  // namespace s_curve_planner
