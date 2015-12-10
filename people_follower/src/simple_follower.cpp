/*
 * simple_follower.cpp
 *
 *  Created on: Sep 28, 2014
 *      Author: lx
 */



#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
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

void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{

	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
cmd_vel.angular.z = 0.0;

  int i=msg->people.size();


  if (i>0) {
	  //Extract coordinates of first detected person
	  double x=msg->people[0].pos.x;
	  double y=msg->people[0].pos.y;
	  //Calculate angle error
	  double AngleError=atan2(y,x);
	  //Calculate distance error
	  double DistanceError=sqrt(pow(x,2)+pow(y,2));

//	  //log
//	  double timemeasure =ros::Time::now().toSec();
//	  double elaspedtime=timemeasure-timepreviousmeasure;
//	  timepreviousmeasure=timemeasure;
//
//	  double distancecoveredperson=sqrt(pow(x-xp,2)+pow(y-yp,2));
//	  xp=x;
//	  yp=y;

	  ROS_INFO("Robot2HumanDistance: %f", DistanceError);
	  ROS_INFO("xperson: %f", x);
	  ROS_INFO("yperson: %f", y);
          ROS_INFO("distance: %f", DistanceTarget);	


	  cmd_vel.angular.z = -AngleError*KpAngle;
	  double linearspeed=(DistanceError-DistanceTarget)*KpDistance;

	  if (linearspeed>MaxSpeed)
	  {
		  linearspeed=MaxSpeed;
	  }

	  if (linearspeed<0){
		  linearspeed=0;
	  }
	  double Cobstacle=1;
	  if (ObstacleFounded==true){
		  Cobstacle=min;
	  }
	   //cmd_vel.linear.x = linearspeed*Cobstacle;
	   cmd_vel.linear.x = linearspeed;
  	  }

  cmd_vel_pub.publish(cmd_vel);
}
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("xrobot: %f", msg->pose.pose.position.x);
	ROS_INFO("yrobot: %f", msg->pose.pose.position.y);
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double alpha_ll=atan2(0.3,0);
	double alpha_lr=atan2(-0.3,0);
	double alpha_ul=atan2(0.3,1);
	double alpha_ur=atan2(-0.3,1);

	int arraylength=msg->ranges.size();

	double angle=-2.09439516068;
	ObstacleFounded=false;
	min=1;
	  for (int i=0; i<arraylength;i++)
	  {
          //test for the left segment
          if (angle < alpha_ll && angle > alpha_ul)
          {
              //If there is an echo smaller than the distance between the origin and the left segment of the rectangle
              if (msg->ranges[i]< (0.3/ sin(angle)))
              {
                  ObstacleFounded = true;
                  if (msg->ranges[i] < min)
                  {
                      min = msg->ranges[i];
                  }
              }
          }

          //test for the right segment
          if (angle < alpha_ur && angle > alpha_lr)
          {
              //If there is an echo smaller than the distance between the origin and the right segment of the rectangle
              if (msg->ranges[i] < (-0.3/ sin(angle)))
              {
                  ObstacleFounded = true;
                  if (msg->ranges[i] < min)
                  {
                      min = msg->ranges[i];
                  }
              }
          }

          //test for the center segment
          if (angle< alpha_ul && angle > alpha_ur)
          {
              //If there is an echo smaller than the distance between the origin and the right segment of the rectangle
              if (msg->ranges[i] < (1 / cos(angle)))
              {
            	  ObstacleFounded = true;
            	                  if (msg->ranges[i] < min)
            	                  {
            	                      min = msg->ranges[i];
            	                  }
              }
          }

		  angle=angle+0.00872664619237;
	  }
}

int main(int argc, char **argv){


	ros::init(argc, argv, "follower_node");
    ros::NodeHandle n;

//	 n.param(std::string("kp_distance"), KpDistance, 1.5);
//	 n.param(std::string("distance_target"), DistanceTarget, 1.3);
//	 n.param(std::string("max_speed"), MaxSpeed, 1.2);

	 cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));

	 ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback);
	 //ros::Subscriber sub2 = n.subscribe("laserscan", 10, laserCallback);
	 ros::Subscriber sub3 = n.subscribe("/RosAria/pose", 10, poseCallback);

	  ros::spin();

	  return 0;
}




