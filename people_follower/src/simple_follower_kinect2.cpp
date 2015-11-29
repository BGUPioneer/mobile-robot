#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>

#define PI 3.14159265

double KpAngle=0.8;
double KpDistance=1.2;
double DistanceTarget=1.2;
double MaxSpeed=0.5;
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
bool ObstacleFounded=false;
double min=1;
double xp=0;
double yp=0;
double timepreviousmeasure=0;



//void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
//{

//    cmd_vel.linear.x = 0.0;
//    cmd_vel.linear.y = 0.0;
//    cmd_vel.linear.z = 0.0;
//    cmd_vel.angular.x = 0.0;
//  cmd_vel.angular.y = 0.0;
//cmd_vel.angular.z = 0.0;

//  int i=msg->people.size();


//  if (i>0) {
//      //Extract coordinates of first detected person
//      double x=msg->people[0].pos.x;
//      double y=msg->people[0].pos.y;
//      //Calculate angle error
//      double AngleError=atan2(y,x);
//      //Calculate distance error
//      double DistanceError=sqrt(pow(x,2)+pow(y,2));

//      ROS_INFO("Robot2HumanDistance: %f", DistanceError);
//      ROS_INFO("xperson: %f", x);
//      ROS_INFO("yperson: %f", y);
//      ROS_INFO("distance: %f", DistanceTarget);


//      cmd_vel.angular.z = -AngleError*KpAngle;
//      double linearspeed=(DistanceError-DistanceTarget)*KpDistance;

//      if (linearspeed>MaxSpeed)
//      {
//          linearspeed=MaxSpeed;
//      }

//      if (linearspeed<0){
//          linearspeed=0;
//      }
//      double Cobstacle=1;
//      if (ObstacleFounded==true){
//          Cobstacle=min;
//      }
//       //cmd_vel.linear.x = linearspeed*Cobstacle;
//       cmd_vel.linear.x = linearspeed;
//      }

//  cmd_vel_pub.publish(cmd_vel);
//}

int main(int argc, char **argv){


    ros::init(argc, argv, "simple_follower_kinect2");
	 ros::NodeHandle n;
	 cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
	 //ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback);
	 ros::spin();
	  return 0;
}


