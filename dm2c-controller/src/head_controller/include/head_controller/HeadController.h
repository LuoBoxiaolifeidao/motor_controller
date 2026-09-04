#ifndef HEAD_CONTROLLER_H
#define HEAD_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <common_comms/HeadSweepAction.h>

#include <mutex>
#include <string>

#include "SCServo.h"

/**
 * Head controller: two Feetech SMS/STS servos (yaw + pitch)
 * Action: common_comms/HeadSweep
 */
class HeadController
{
public:
  explicit HeadController(ros::NodeHandle& nh);
  ~HeadController();

private:
  struct AxisConfig
  {
    int    id         = 1;
    double base_deg   = 90.0;
    double offset_min = -45.0;
    double offset_max = 45.0;
    int    speed      = 60;
    int    accel      = 60;
    int    tolerance  = 15;     // arrival tolerance (steps)
  };

  int    valueToPos(const AxisConfig& ax, double value_deg) const;
  double posToValue(const AxisConfig& ax, int pos) const;

  bool initServos();
  void loadConfig();
  void goHome();
  void executeCb(const common_comms::HeadSweepGoalConstPtr& goal);

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<common_comms::HeadSweepAction> as_;

  SMS_STS      servo_;
  std::mutex   bus_mutex_;

  std::string  port_;
  int          baudrate_   = 115200;

  AxisConfig   yaw_;
  AxisConfig   pitch_;

  double       home_yaw_   = 0.0;
  double       home_pitch_ = 0.0;

  static constexpr double kCountsPerDeg = 4096.0 / 360.0;
};

#endif  // HEAD_CONTROLLER_H
