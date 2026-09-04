#include <ros/ros.h>
#include "DM2CNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dm2c_controller");
  DM2CNode node;
  ros::spin();
  return 0;
}
