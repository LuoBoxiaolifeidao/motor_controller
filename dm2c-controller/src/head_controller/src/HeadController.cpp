#include "HeadController.h"

#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

HeadController::HeadController(ros::NodeHandle& nh)
  : nh_(nh),
    as_(nh_, "head_sweep",
        boost::bind(&HeadController::executeCb, this, _1), false)
{
  loadConfig();

  if (!initServos())
  {
    ROS_ERROR("[head_controller] servo init failed, action server not started");
    return;
  }

  goHome();

  as_.start();
  ROS_INFO("[head_controller] started, action: ~head_sweep (yaw id=%d, pitch id=%d)",
           yaw_.id, pitch_.id);
}

HeadController::~HeadController()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  servo_.end();
}

void HeadController::loadConfig()
{
  std::string path = ros::package::getPath("head_controller") + "/config/head.yaml";
  try {
    YAML::Node cfg = YAML::LoadFile(path);
    ROS_INFO("[head_controller] config loaded: %s", path.c_str());

    if (cfg["port"])          port_        = cfg["port"].as<std::string>();
    if (cfg["baudrate"])      baudrate_    = cfg["baudrate"].as<int>();

    auto readAxis = [](const YAML::Node& n, AxisConfig& ax) {
      if (!n) return;
      if (n["id"])         ax.id         = n["id"].as<int>();
      if (n["base_deg"])   ax.base_deg   = n["base_deg"].as<double>();
      if (n["offset_min"]) ax.offset_min = n["offset_min"].as<double>();
      if (n["offset_max"]) ax.offset_max = n["offset_max"].as<double>();
      if (n["speed"])      ax.speed      = n["speed"].as<int>();
      if (n["accel"])      ax.accel      = n["accel"].as<int>();
      if (n["tolerance"])  ax.tolerance  = n["tolerance"].as<int>();
    };
    readAxis(cfg["yaw"],   yaw_);
    readAxis(cfg["pitch"], pitch_);

    if (cfg["home"]) {
      if (cfg["home"]["yaw"])   home_yaw_   = cfg["home"]["yaw"].as<double>();
      if (cfg["home"]["pitch"]) home_pitch_ = cfg["home"]["pitch"].as<double>();
    }
  } catch (const YAML::Exception& e) {
    ROS_ERROR("[head_controller] failed to load %s: %s", path.c_str(), e.what());
  }
}

bool HeadController::initServos()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!servo_.begin(baudrate_, port_.c_str()))
  {
    ROS_ERROR("[head_controller] cannot open serial: %s @ %d", port_.c_str(), baudrate_);
    return false;
  }

  servo_.EnableTorque(yaw_.id, 1);
  servo_.EnableTorque(pitch_.id, 1);

  ROS_INFO("[head_controller] serial OK: %s @ %d", port_.c_str(), baudrate_);
  return true;
}

void HeadController::goHome()
{
  const int yaw_target   = valueToPos(yaw_,   home_yaw_);
  const int pitch_target = valueToPos(pitch_, home_pitch_);

  ROS_INFO("[head_controller] going home: yaw=%.1f (pos=%d), pitch=%.1f (pos=%d)",
           home_yaw_, yaw_target, home_pitch_, pitch_target);

  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    servo_.WritePosEx(yaw_.id,   yaw_target,   yaw_.speed,   yaw_.accel);
    servo_.WritePosEx(pitch_.id, pitch_target, pitch_.speed, pitch_.accel);
  }

  ros::Rate rate(20.0);
  while (ros::ok())
  {
    int yaw_pos, pitch_pos;
    {
      std::lock_guard<std::mutex> lock(bus_mutex_);
      yaw_pos   = servo_.ReadPos(yaw_.id);
      pitch_pos = servo_.ReadPos(pitch_.id);
    }
    if (yaw_pos < 0 || pitch_pos < 0) break;

    if (std::abs(yaw_pos - yaw_target) <= yaw_.tolerance &&
        std::abs(pitch_pos - pitch_target) <= pitch_.tolerance)
    {
      ROS_INFO("[head_controller] home reached (yaw=%d, pitch=%d)", yaw_pos, pitch_pos);
      return;
    }
    rate.sleep();
  }
  ROS_WARN("[head_controller] home move interrupted");
}

int HeadController::valueToPos(const AxisConfig& ax, double value_deg) const
{
  double off = std::clamp(value_deg, ax.offset_min, ax.offset_max);
  double deg = ax.base_deg + off;
  int pos = static_cast<int>(std::lround(deg * kCountsPerDeg));
  return std::clamp(pos, 0, 4095);
}

double HeadController::posToValue(const AxisConfig& ax, int pos) const
{
  return pos / kCountsPerDeg - ax.base_deg;
}

void HeadController::executeCb(const common_comms::HeadSweepGoalConstPtr& goal)
{
  common_comms::HeadSweepResult result;

  const int yaw_target   = valueToPos(yaw_,   goal->yaw);
  const int pitch_target = valueToPos(pitch_, goal->pitch);

  ROS_INFO("[head_controller] target yaw=%.1f (pos=%d), pitch=%.1f (pos=%d)",
           goal->yaw, yaw_target, goal->pitch, pitch_target);

  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    servo_.WritePosEx(yaw_.id,   yaw_target,   yaw_.speed,   yaw_.accel);
    servo_.WritePosEx(pitch_.id, pitch_target, pitch_.speed, pitch_.accel);
  }

  ros::Rate rate(20.0);

  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      result.error_code = -3;
      result.message    = "preempted";
      as_.setPreempted(result);
      return;
    }

    int yaw_pos, pitch_pos;
    {
      std::lock_guard<std::mutex> lock(bus_mutex_);
      yaw_pos   = servo_.ReadPos(yaw_.id);
      pitch_pos = servo_.ReadPos(pitch_.id);
    }

    if (yaw_pos < 0 || pitch_pos < 0)
    {
      result.error_code = -1;
      result.message    = "read pos failed";
      as_.setAborted(result);
      ROS_ERROR("[head_controller] read pos failed (yaw=%d, pitch=%d)", yaw_pos, pitch_pos);
      return;
    }

    common_comms::HeadSweepFeedback fb;
    fb.current_yaw   = posToValue(yaw_,   yaw_pos);
    fb.current_pitch = posToValue(pitch_, pitch_pos);
    as_.publishFeedback(fb);

    if (std::abs(yaw_pos - yaw_target) <= yaw_.tolerance &&
        std::abs(pitch_pos - pitch_target) <= pitch_.tolerance)
    {
      result.error_code = 0;
      result.message    = "ok";
      as_.setSucceeded(result);
      return;
    }

    rate.sleep();
  }
}
