#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_param");

  ros::NodeHandle n;

  int myi;
  double myd;

  n.param("test_param/myInt", myi, 1);
  n.param("test_param/myDouble", myd, 1.1);

  ROS_INFO("Float: %d", myi);
  ROS_INFO("Double: %f", myd);

  ros::spin();

  return 0;
}
