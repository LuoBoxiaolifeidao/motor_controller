#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_ik_solver/left_arm_ik.hpp"
#include "arm_ik_solver/right_arm_ik.hpp"

using KDL::Vector;
using KDL::Rotation;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ActionClient = rclcpp_action::Client<FollowJointTrajectory>;

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

enum class Side { LEFT, RIGHT };

class DualArmNode : public rclcpp::Node {
public:
  DualArmNode()
  : Node("dual_arm_node")
  {
    left_arm_client_     = rclcpp_action::create_client<FollowJointTrajectory>(this, "/left_arm_controller/follow_joint_trajectory");
    right_arm_client_    = rclcpp_action::create_client<FollowJointTrajectory>(this, "/right_arm_controller/follow_joint_trajectory");
    left_gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/left_gripper_controller/follow_joint_trajectory");
    right_gripper_client_= rclcpp_action::create_client<FollowJointTrajectory>(this, "/right_gripper_controller/follow_joint_trajectory");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    using Msg = std_msgs::msg::Float64MultiArray;
    left_tcp_sub_  = create_subscription<Msg>("/left_tcp", 10,
      [this](Msg::ConstSharedPtr m){ tcp_callback(m, Side::LEFT); });
    right_tcp_sub_ = create_subscription<Msg>("/right_tcp", 10,
      [this](Msg::ConstSharedPtr m){ tcp_callback(m, Side::RIGHT); });
    left_tool_sub_ = create_subscription<Msg>("/left_tool", 10,
      [this](Msg::ConstSharedPtr m){ tool_callback(m, Side::LEFT); });
    right_tool_sub_ = create_subscription<Msg>("/right_tool", 10,
      [this](Msg::ConstSharedPtr m){ tool_callback(m, Side::RIGHT); });

    RCLCPP_INFO(get_logger(), "Dual IK node (4 controllers) started");
  }

private:
  void tcp_callback(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg, Side side) {
    if (msg->data.size() < 6) return;
    Vector pos(msg->data[0], msg->data[1], msg->data[2]);
    Rotation rot = Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);

    auto& q = (side == Side::LEFT) ? left_q_ : right_q_;
    const auto& jl = (side == Side::LEFT) ? left_arm_ik::JOINT_LOWER : right_arm_ik::JOINT_LOWER;
    const auto& ju = (side == Side::LEFT) ? left_arm_ik::JOINT_UPPER : right_arm_ik::JOINT_UPPER;

    std::array<double,8> q_init = {q[0],q[1],q[2],q[3],q[4],
      clamp(msg->data[3],jl[5],ju[5]),
      clamp(msg->data[4],jl[6],ju[6]),
      clamp(msg->data[5],jl[7],ju[7])};

    const char* label = (side == Side::LEFT) ? "Left" : "Right";
    RCLCPP_INFO(get_logger(), "%s TCP: [%.3f %.3f %.3f]", label, msg->data[0],msg->data[1],msg->data[2]);

    if (side == Side::LEFT) {
      auto res = left_arm_ik::solve_ik(pos, rot, q_init);
      if (res.success) {
        q = res.q8;
        auto fk = left_arm_ik::fk(res.q8);
        double r,pw,yw; fk.M.GetRPY(r,pw,yw);
        RCLCPP_INFO(get_logger(), "L OK pos=[%.3f,%.3f,%.3f] e_p=%.4f e_r=%.4f",
          fk.p.x(),fk.p.y(),fk.p.z(), res.pos_err,res.rot_err);
        send_trajectory(left_arm_client_, {
          "l_post_Joint","l_shoulder_Joint","l_arm_JointA","l_arm_JointB","l_arm_JointC",
          "l_wrist_p_Joint","l_wrist_y_Joint","l_wrist_r_Joint"},
          {q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]});
      } else { RCLCPP_WARN(get_logger(), "L FAIL e_p=%.4f e_r=%.4f", res.pos_err, res.rot_err); }
    } else {
      auto res = right_arm_ik::solve_ik(pos, rot, q_init);
      if (res.success) {
        q = res.q8;
        auto fk = right_arm_ik::fk(res.q8);
        double r,pw,yw; fk.M.GetRPY(r,pw,yw);
        RCLCPP_INFO(get_logger(), "R OK pos=[%.3f,%.3f,%.3f] e_p=%.4f e_r=%.4f",
          fk.p.x(),fk.p.y(),fk.p.z(), res.pos_err,res.rot_err);
        send_trajectory(right_arm_client_, {
          "r_post_Joint","r_shoulder_Joint","r_arm_JointA","r_arm_JointB","r_arm_JointC",
          "r_wrist_p_Joint","r_wrist_y_Joint","r_wrist_r_Joint"},
          {q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]});
      } else { RCLCPP_WARN(get_logger(), "R FAIL e_p=%.4f e_r=%.4f", res.pos_err, res.rot_err); }
    }
  }

  void tool_callback(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg, Side side) {
    if (msg->data.size() != 2) return;
    if (side == Side::LEFT) {
      send_trajectory(left_gripper_client_,
        {"l-hand-l-finger_Joint","l-hand-r-finger_Joint"},
        {msg->data[0], msg->data[1]});
    } else {
      send_trajectory(right_gripper_client_,
        {"r-hand-l-finger_Joint","r-hand-r-finger_Joint"},
        {msg->data[0], msg->data[1]});
    }
  }

  void send_trajectory(ActionClient::SharedPtr client,
                        const std::vector<std::string>& joints,
                        const std::vector<double>& positions) {
    if (!client->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN(get_logger(), "Action server not available"); return;
    }
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = joints;
    goal.trajectory.points.resize(1);
    auto& pt = goal.trajectory.points[0];
    pt.positions = positions;
    pt.time_from_start.sec = 2;
    client->async_send_goal(goal);
  }

  ActionClient::SharedPtr left_arm_client_;
  ActionClient::SharedPtr right_arm_client_;
  ActionClient::SharedPtr left_gripper_client_;
  ActionClient::SharedPtr right_gripper_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_tool_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_tool_sub_;

  std::array<double,8> left_q_  = {0.1,0,0,0,0,0,0,0};
  std::array<double,8> right_q_ = {0.1,0,0,0,0,0,0,0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualArmNode>());
  rclcpp::shutdown();
  return 0;
}
