#include <ros/ros.h>
#include "HeadController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_controller");
  ros::NodeHandle nh;

  HeadController node(nh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
