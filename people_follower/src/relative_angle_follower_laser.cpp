/*
 * relative_angle_follower.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: Hanan Zaichyk
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

//parameters should be chnaged to suit the experiment goal.
double MaxSpeed=0.5; // Maximum speed the robot can drive. the speed is m/s 
double KpDistance=0.8; // Ratio between the speed of the robot and the distance to the robot current target. (speed = kp*distance)
                        // higher Kp means faster accleration, and more aggresive movements
double KpAngle=0.8; // same as KpDistance, only for the angular velocity.

//parameters should be changed according to experiment/human goal
double followingAngle = PI/3; //the angle the robot should follow the human, in Radians
double DistanceTarget=1;      //the distance the robot should keep to human, in Meters

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
//this callback, receive a msg of a person position and decide how fast and which direction the robot should go.

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    // the Pioneer Robot can move only in the x-linear and z-angular axises. 
    // If changed to a different robot which can move in other axisex, they should be 0 to.

  int i=msg->people.size();
  if (i>0) {
      //Extract coordinates of first detected person
      
      //gets the x,y coordinates of the person, relative to robot. important. x is the 
    double y=msg->people[0].pos.y;
    double x=msg->people[0].pos.x;

      // for debugging and comparisons in the experiments
      ROS_INFO("Person position: %f,%f     Angle: %f", -y,x, (atan2(x,-y)*180/PI));
      //end of debugging session

      // here we shift the coordinates so the robot will navigate to (x,y) position in the following angle to the human position.
      double shiftY = y - sin(followingAngle)*DistanceTarget;// Y is the left-rigth axis, when left side is possitive
      double shiftX = x - cos(followingAngle)*DistanceTarget; // X is the forward-backward axis, when forward is possitive. theres Robot Deiates in +0.4!!!

      //Calculate angle difference from target
      double AngleError=atan2(shiftY,shiftX);
      //Calculate distance from target
      double DistanceError=sqrt(pow(shiftX,2)+pow(shiftY,2));

      //double linearspeed=(DistanceError)*KpDistance;  // this is also posebilty. the down one chosen for more astetic movement
      double linearspeed = (shiftX)*KpDistance; // the distance to target consider only the front distance. 
                                                //so the robot will never pass it targer and will need to go backwards by rotating
                                                // for now this works pretty good to prevent overshooting. next work could learn from P/PID/PD controller
      double angularspeed = (AngleError*KpAngle);

      if (linearspeed>MaxSpeed)
      {
          linearspeed=MaxSpeed;
      }

      if (linearspeed<0){  // means the robot passed the wanted location - so stop
          linearspeed=0;
      }

      if (linearspeed<0.1 and  abs(90+180*followingAngle/PI-angle)<10)  angularspeed=0; // important, because the robot will never be exactly at the right spot and orientation. stops the rotation when robot is placed at right x,y position and rotation is close enough
       cmd_vel.linear.x = linearspeed;
       cmd_vel.angular.z = AngleError*KpAngle;
	}
  cmd_vel_pub.publish(cmd_vel); 
}


int main(int argc, char **argv){

  // ros command : initialization of this node
  ros::init(argc, argv, "follower_node"); 
  ros::NodeHandle n; //
 
//TODO: find how to allow setting this parameters using launch file
//	 n.param(std::string("kp_distance"), KpDistance, 1.5);
//	 n.param(std::string("distance_target"), DistanceTarget, 1.3);
//	 n.param(std::string("max_speed"), MaxSpeed, 1.2);

  cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2)); // diclaring of a publisher, publishes command of velocity to robot 
  ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback); // getting information from robot about people location
  ros::spin();  // ros command : repeating main function until external interfirence

  return 0;
}