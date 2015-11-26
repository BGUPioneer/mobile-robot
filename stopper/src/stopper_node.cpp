#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "rosaria/BumperState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_listener.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "std_srvs/Empty.h"


ros::Publisher stop_message;
ros::Publisher stop_twist;
geometry_msgs::Twist zero_twist;
tf::TransformListener *tf_listener;

//Bumper Callback
void BumperCallback(const rosaria::BumperState::ConstPtr& msg)
{
  bool stop_bool=false;
  for (int i=0; (i<msg->front_bumpers.size()) && (stop_bool==false) ;i++){
      if (msg->front_bumpers[i]){
          ROS_INFO("Bumper touched:[%i]", i);
          stop_bool=true;
          stop_twist.publish(zero_twist);
      }
  }

  std_msgs::Bool stop;
  stop.data=stop_bool;
  stop_message.publish(stop);

}

float DistanceThreshold=1;
float WidthThreshold=0.5;
void LaserCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  bool stop_bool=false;

  //tf_listener->waitForTransform("/laser_frame", (*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));

  //(*tf_listener).transformPointCloud("/laser_frame", *msg, pc_out);

  for (int i=0; (i<(*msg).points.size()) && (stop_bool==false) ;i++)
  {
      if (((*msg).points[i].x < DistanceThreshold && (*msg).points[i].x >0) && (std::abs((*msg).points[i].y) < WidthThreshold/2))
      {
          ROS_INFO("Obstacle detected by laser at : x:%f y:%f", (*msg).points[i].x , (*msg).points[i].y);
          stop_bool=true;
          stop_twist.publish(zero_twist);
  }
  std_msgs::Bool stop;
  stop.data=stop_bool;
  stop_message.publish(stop);

  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stopper");

  ros::NodeHandle n;

  zero_twist.linear.x=0;
  zero_twist.linear.y=0;
  zero_twist.angular.z=0;

  tf_listener = new tf::TransformListener();
  ros::Subscriber sub_bumper = n.subscribe("/RosAria/bumper_state", 10, BumperCallback);
  ros::Subscriber sub_laser = n.subscribe("/RosAria/S3Series_1_pointcloud", 10, LaserCallback);
  stop_message=ros::Publisher(n.advertise<std_msgs::Bool> ("stopper/emergency_stop",2));
  stop_twist=ros::Publisher(n.advertise<geometry_msgs::Twist> ("stopper/stop_cmd_vel",2));

  ros::spin();

  return 0;
}
