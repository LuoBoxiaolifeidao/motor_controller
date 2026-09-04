#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <kdl/frames.hpp>

namespace arm_ik_solver {

struct PoseTarget
{
  KDL::Vector pos;
  KDL::Rotation rot;
  bool valid = false;
};

class DualArmIKController : public controller_interface::ControllerInterface
{
public:
  DualArmIKController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void left_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void right_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void dual_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void process_left(const KDL::Vector & pos, const KDL::Rotation & rot);
  void process_right(const KDL::Vector & pos, const KDL::Rotation & rot);

  static double step_toward(double cur, double tgt, double step);

  static constexpr size_t NUM_JOINTS = 6;
  static constexpr size_t TOTAL_JOINTS = 12;

  std::array<std::string, NUM_JOINTS> left_joint_names_;
  std::array<std::string, NUM_JOINTS> right_joint_names_;
  std::vector<double> target_;
  std::vector<double> smoothed_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseTarget>> rt_left_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseTarget>> rt_right_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>> rt_joint_target_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dual_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;

  double interp_step_;
};

}  // namespace arm_ik_solver
