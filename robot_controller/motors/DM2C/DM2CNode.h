#pragma once

#include <functional>
#include <mutex>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <robot_controller/MoveAction.h>
#include <robot_controller/GetState.h>
#include <robot_controller/SetZero.h>

#include "DM2CController.h"

class DM2CNode
{
public:
  DM2CNode();

private:
  void executeCB(const robot_controller::MoveGoalConstPtr& goal);

  bool executeLeft(const robot_controller::MoveGoalConstPtr& goal);
  bool executeRight(const robot_controller::MoveGoalConstPtr& goal);
  bool waitAndPublishFeedback(std::shared_ptr<DM2CController> controller,
                               const robot_controller::MoveGoalConstPtr& goal);
  bool publishFeedback(std::shared_ptr<DM2CController> controller);
  bool getStateCB(robot_controller::GetState::Request&, robot_controller::GetState::Response& res);
  void stopCB(const std_msgs::String::ConstPtr &msg);
  bool setZeroCB(robot_controller::SetZero::Request &req, robot_controller::SetZero::Response &res);
  std::shared_ptr<DM2CController> getCtrl(const std::string& name);

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robot_controller::MoveAction> as_;
  ros::ServiceServer state_srv_;
  ros::Subscriber stop_sub_;
  ros::ServiceServer zero_srv_;

  std::shared_ptr<DM2CController> leftController_;
  std::shared_ptr<DM2CController> rightController_;

  std::mutex stop_mutex_;
  bool stop_requested_ = true;
};
