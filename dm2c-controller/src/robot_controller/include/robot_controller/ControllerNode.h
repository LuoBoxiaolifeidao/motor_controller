#pragma once

#include <vector>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h> 
#include <robot_controller/SetZero.h>
#include <actionlib/server/simple_action_server.h>
#include <common_comms/MoveArmAction.h>
#include <common_comms/GripperCommandAction.h>
#include <unordered_map>

#include "SCServo.h"
#include "SMS_STS_Shared.h"

#include "DM2CController.h"
#include "MWD485Controller.h"
#include "SCurvPlan.h"
#include "MotorConfig.h"
#include "left_arm_ik.h"

enum class Joint {
    Lift    = 0,
    Shoulder,
    Extend,
    Wrist1,
    Wrist2,
    Wrist3,
    COUNT
};

enum class Motor {
    DM2C,
    MWD_Shoulder,
    MWD_Extend,
    FT
};

struct JointState {
    double position = 0.0;
    double velocity = 0.0;
};

class ControllerNode
{
public:
  ControllerNode();
  ~ControllerNode();

private:
  void loadMotorConfig();
  void init();
  void loop();
  void stopAll();
  void readPdo(Motor);
  void writePdo(Motor);
  void publishState();

  void ArmJointsCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void TcpCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
  bool setZeroCB(robot_controller::SetZero::Request &req, robot_controller::SetZero::Response &res);

  std::vector<double> mapIkToJoints(const std::array<double, 8>& q8) const;
  void driveJoints(const std::vector<double>& target);
  void driveJoints(const std::vector<double>& target, common_comms::MoveArmFeedback& feedback);
  void executeMoveArm(const common_comms::MoveArmGoalConstPtr& goal);
  void executeGripper(const common_comms::GripperCommandGoalConstPtr& goal);

private:
  static constexpr int kJointNum = static_cast<int>(Joint::COUNT);
  std::atomic<int> gripper_position_{0};

  KDL::Frame left_tcp_;
  KDL::Frame left_tool_;
  KDL::Frame left_tool_offset_;

  ros::NodeHandle nh_;
  ros::Subscriber arm_joints_sub_;
  ros::Subscriber tcp_sub_;
  ros::ServiceServer zero_srv_;
  ros::Publisher joint_state_pub_;

  std::atomic<bool> running_{false};

  std::thread loop_thread_;

  std::mutex cv_mutex_;
  std::condition_variable cv_;
  std::mutex ft_mutex_;
  std::mutex dm2c_mutex_;
  std::mutex curState_mutex_;
  std::mutex move_mutex_;

  std::shared_ptr<DM2CController>     jointDM2C_;
  std::shared_ptr<MWD485Controller>   jointShoulder_;
  std::shared_ptr<MWD485Controller>   jointExtend_;
  std::shared_ptr<SMS_STS_Shared>      jointFT_;

  DM2CConfig    dm2c_cfg_;
  MWD485Config  shoulder_cfg_;
  MWD485Config  extend_cfg_;
  FTConfig      ft_cfg_;
  GripperConfig gripper_cfg_;

  std::vector<double> minPos_ = {0,   0,  0,   -180, -90,  -180};
  std::vector<double> maxPos_ = {950, 90, 360, 180,  90,   180};

  std::vector<JointState> curState_;
  std::vector<double>     targetPos_;
  std::vector<double>     targetVel_;
  std::vector<double>     targetAcc_;
  std::vector<double>     finalTarget_;

  std::pair<double, uint32_t> dm2c_value_;

  std::array<double, 8> ik_q_ = {0.1, 0, 0, 0, 0, 0, 0, 0};

  typedef actionlib::SimpleActionServer<common_comms::MoveArmAction> MoveArmServer;
  std::shared_ptr<MoveArmServer> move_arm_as_;
  std::unordered_map<std::string, std::vector<double>> named_targets_;
  std::atomic<bool> move_active_{false};

  typedef actionlib::SimpleActionServer<common_comms::GripperCommandAction> GripperServer;
  std::shared_ptr<GripperServer> gripper_as_;
};
