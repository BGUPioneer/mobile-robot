
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "rosaria/BumperState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_listener.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "std_srvs/Empty.h"
#include <obstacles/laserObstacles.h>

//geometry_msgs::Twist zero_twist;
obstacles::laserObstacles ob_msg;

class LaserObstacles
{
    ros::NodeHandle n;
    tf::TransformListener tf_listener;

    ros::Subscriber cmdVel = n.subscribe("/cmd_vel", 10, &LaserObstacles::velocityCallback, this);
    ros::Subscriber sub_laser = n.subscribe("/RosAria/S3Series_1_pointcloud", 10, &LaserObstacles::LaserCallback,this);
    ros::Publisher pub=(n.advertise<obstacles::laserObstacles> ("/obstacles/laserObstacles",10));

    double DistanceCheck=1.5;  //in front of the robot
    double WidthCheck= 0.4;  //for each side
    double angularVelocity;
    double linearVelocity;
    double angularCommand;
    double XclosestObstacle=100.0;
    double YclosestObstacle=100.0;
    double DclosestObstacle=100.0;


    void LaserCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  sensor_msgs::PointCloud pc_out;
  bool obstacle=false;

  tf_listener.waitForTransform("/laser_frame", (*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
  tf_listener.transformPointCloud("/laser_frame", *msg, pc_out);

  for (int i=0; i<pc_out.points.size() ;i++)
  {
      if (pc_out.points[i].x < DistanceCheck && pc_out.points[i].x >0)
      {
          //2 conditions: 1. y smaller than positive WidthCheck multipile the power of the turn of the robot; 2.y bigger than negative WidthCheck multipile the power of the turn of the robot;
          if ((pc_out.points[i].y < WidthCheck*(1+angularVelocity)) && (pc_out.points[i].y > -WidthCheck*(1-angularVelocity))){
          ROS_INFO("angularVelocity :%f", angularVelocity);
          ROS_INFO("Obstacle detected by laser at : x:%f y:%f", pc_out.points[i].x , pc_out.points[i].y);

            double pointDistance=sqrt(pow(pc_out.points[i].x,2)+pow(pc_out.points[i].y,2));
           //to get the closet obstacle
            if (pointDistance<DclosestObstacle){
                XclosestObstacle=pc_out.points[i].x;
                YclosestObstacle=pc_out.points[i].y;
                DclosestObstacle=pointDistance;
            }
          obstacle=true;
        }
      }
  }
  //if obstacle from the left than turn right
  if (YclosestObstacle>=0){
  angularCommand=-(WidthCheck-YclosestObstacle);
  }
  //if obstacle from the right than turn left
  else {
      angularCommand=WidthCheck+YclosestObstacle;
  }
  //publish the variables
    ob_msg.detect_obstacles=obstacle;
    ob_msg.angular_velocity= angularCommand;
    pub.publish(ob_msg);
}


void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    angularVelocity=msg->angular.z;
    linearVelocity=msg->linear.x;
}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_obstacles_avoidance");

  LaserObstacles LO;
  ros::NodeHandle n;

  ros::spin();
  return 0;
}


