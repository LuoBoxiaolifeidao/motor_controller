#include <ros/ros.h>
#include "ControllerNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dm2c_controller");
  ControllerNode node;
  ros::AsyncSpinner spinner(2); // 2 个 ROS 回调线程
  spinner.start();

  ros::waitForShutdown();
  return 0;
}
