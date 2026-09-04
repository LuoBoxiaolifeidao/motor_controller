#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/subscription.hpp>

#include "mppi_controller/mppi_planner.hpp"

namespace mppi_controller {

class MPPIController : public controller_interface::ControllerInterface {
public:
  MPPIController();

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

  static bool is_prismatic(size_t joint_idx) {
    int idx = static_cast<int>(joint_idx % ARM_JOINTS);
    return idx == 0 || idx == 2 || idx == 3 || idx == 4;
  }

  static constexpr size_t ARM_JOINTS = 8;
  static constexpr size_t TOTAL_JOINTS = 16;
  static constexpr double DEAD_ZONE_REV = 0.008726;
  static constexpr double DEAD_ZONE_PRIS = 0.001;

  std::array<std::string, ARM_JOINTS> left_names_;
  std::array<std::string, ARM_JOINTS> right_names_;
  std::array<double, ARM_JOINTS> left_q_init_;
  std::array<double, ARM_JOINTS> right_q_init_;
  std::array<double, ARM_JOINTS> left_q_target_;
  std::array<double, ARM_JOINTS> right_q_target_;

  std::mutex mtx_;
  std::unique_ptr<MPPIPlanner> mppi_;
  MPPIConfig mppi_cfg_;

  std::vector<double> q_min_, q_max_, u_min_, u_max_;
  bool active_ = false;
  bool left_has_target_ = false;
  bool right_has_target_ = false;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dual_sub_;
};

}  // namespace mppi_controller
