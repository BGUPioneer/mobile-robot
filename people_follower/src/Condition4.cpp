/*
 * relative_angle_follower.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: Hanan Zaichyk // being amended for dyanmic speed following
 */


// Condition 1 distance - accleration - angle (0.5-0.5-30)
 // modified to 1.2 - 1.0 - 30


#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "ros/ros.h"   // import rospy
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h" // from geometry_msgs.msg import Twist
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>

#define PI 3.14159265

//parameters should be chnaged to suit the experiment goal.
double MaxSpeed=1.0; // Maximum speed the robot can drive. the speed is m/s (formerly 0.8) 
double KpDistance = 1.0; // Ratio between the speed of the robot and the distance to the robot current target. (speed = kp*distance)
                        // higher Kp means faster accleration, and more aggresive movements (formerly 1)
double KpAngle=1.0; // same as KpDistance, only for the angular velocity.

//parameters should be changed according to experiment/human goal
double followingAngle = PI/6; //the angle the robot should follow the human, in Radians
double DistanceTarget=1.2;      //the distance the robot should keep to human, in Meters


// rospy.init_node('human_follower')
ros::Publisher cmd_vel_pub;   // pub = rospy.Publisher('/cmd_vel',Twist, queue_size =1)
geometry_msgs::Twist cmd_vel;  // 

void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
//this callback, receive a msg of a person position and decide how fast and which direction the robot should go.
// move = Twist()
    cmd_vel.linear.x = 0.0;  //move.linear.x = 0.0
    //cmd_vel.linear.y = 0.0;
    //cmd_vel.linear.z = 0.0;
    //cmd_vel.angular.x = 0.0;
    //cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0; //move.angular.z = 0.0

  int i=msg->people.size();  // number of people being detected


  if (i>0) {
      //Extract coordinates of first detected person
      
      //gets the x,y coordinates of the person, relative to robot. important: x is the forward axis, y is side
    double y=msg->people[0].pos.y;
    double x=msg->people[0].pos.x;

      // for debugging and comparisons in the experiments
    double angle = (atan2(x,-y)*180/PI);
      ROS_INFO("Person position: %f,%f     Angle: %f Distance:%f", -y,x, angle , ( sqrt(pow(x,2)+pow(y,2)) ));
      //end of debugging session

      // here we shift the coordinates so the robot will navigate to (shiftx,shifty) position in the following angle to the human position.
      double shiftY = y - sin(followingAngle)*DistanceTarget;// Y is the left-rigth axis, when left side is possitive
      double shiftX = x - cos(followingAngle)*DistanceTarget; // X is the forward-backward axis, when forward is possitive. theres is a +0.4 because of robot deviation
      ROS_INFO("Going to: %f,%f",-shiftY,shiftX);
      //Calculate angle error
      double AngleError=atan2(shiftY,shiftX);
      //Calculate distance error
      double DistanceError=sqrt(pow(shiftX,2)+pow(shiftY,2));


      
      double linearspeed = (shiftX)*KpDistance;
      
      double angularspeed = (AngleError*KpAngle);

      if (linearspeed>MaxSpeed)
      {
          linearspeed=MaxSpeed;
      }

      if (linearspeed<0){ // means the robot passed the wanted location - so stop
          linearspeed=0;
      }
      if (linearspeed<0.1 and  abs(90+180*followingAngle/PI-angle)<10)  angularspeed=0; 
      
      //if (abs(angularspeed)>2*linearspeed and angle > 0) angularspeed = 2*angularspeed/(abs(angularspeed))*linearspeed;
     /* if(linearspeed=0){
        if (angle>0){
          if (angle <90+followingAngle) angularspeed = -1;
          else angularspeed =1;
        }
        else{
          if (angle>-90) angularspeed = -2;
          else angularspeed = 2;
        }
      }*/
       cmd_vel.linear.x = linearspeed; // move.linear.x = linearspeed
       
       cmd_vel.angular.z = angularspeed; // move.angular.z = angularspeed

      /* suppose to check the error(the distance from wanted place)during following 
      see general thoughts in the end for explanation why it is irelevant
      ROS_INFO("speed: %f, %f",linearspeed,angularspeed);
      if (DistanceError>0.2) ROS_INFO("BAD!!!");
      else ROS_INFO("GOOOODDDDD");
      */

	}
  cmd_vel_pub.publish(cmd_vel);  // pub.publish(move)
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

// This is not important for this node () , we transfer the information of orientation from quaternions to radians and then degress
    //double Xr =  msg->pose.pose.position.x;
    //double Yr =  msg->pose.pose.position.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    z = 2*asin(z)*180/PI;
    w = 2*acos(w)*180/PI;
    double orientation;
    if (z>0) orientation = w;
    else orientation = 360-w;
    //ROS_INFO("Robot position: %f ",orientation);
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "follower_node"); // Tells ROS that this is a new node
    ros::NodeHandle n;                      // just for convinient later on
// TODO: find how to allow setting this parameters using launch file
//	 n.param(std::string("kp_distance"), KpDistance, 1.5);
//	 n.param(std::string("distance_target"), DistanceTarget, 1.3);
//	 n.param(std::string("max_speed"), MaxSpeed, 1.2);

    cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2)); // velocity commands to robot
    
    ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback); // position of the person
    ros::Subscriber sub3 = n.subscribe("/RosAria/pose", 10, poseCallback); // position of the robot  


    // while not rospy.is_shutdown()
    //rate.sleep()
    ros::spin(); // Keeps looping on the main until interupted from user (using Ctrl-c)  //rospy.spin()

      return 0;
}


/* General thoughts and TODO about this algorithm:
General:
The algorithm is seems fine in subjective eyes.
The robot follows the human at relatively study pace, and get to the right place at stops at most cases.
The main weaknes is the objective position of the robot relative to human. The angle output is far from
wanted angle (10-20 degrees on straight line, on turns it gets off track). This is understandable since 
the algorithms is reactive to the human walking there for calculates and goes relative to where the human 
was and not where he is going to be. Since this algorithm is naive it cannot conclude about the speed and 
orientation of person in space, there for cannot be flexible (acting deferent when human is walking or 
standing in place).
For now I dont think this algorithm is suitable for achiving accurate following angle when walking,
but it is very goog for subjective feeling of following in wanted angle, and to wait for person at wanted
position when person stops.
(Can be very good for purposes of porterage, like following lugagae\toolbot\shopping cart etc.)

TODO:

*/