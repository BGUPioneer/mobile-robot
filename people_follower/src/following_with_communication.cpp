/*
 * following_with_communication.cpp
 *
 *  Created on: Dec 10, 2018
 *      Author: Samuel 
 */

#include "stdio.h"
#include "math.h"
#include "ros/ros.h"   // import rospy
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h" // from geometry_msgs.msg import Twist
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>
#include <sound_play/sound_play.h>
#include <unistd.h>
#define PI 3.14159265

//parameter definition
double max_speed=1.5; // Not more than 1.5m/s 
double acceleration_coeff = 0.8; //acceleration // ideal 0.8
double angular_vel=0.5; //velocity during turns
double following_angle = 0/PI; //the angle the robot keeps behind the person, in radians
double following_dist=0.6;      //the distance between the robot and person, in meters


// rospy.init_node('human_follower')
ros::Publisher cmd_vel_pub;   // pub = rospy.Publisher('/cmd_vel',Twist, queue_size =1)
geometry_msgs::Twist cmd_vel;  // 


// **function for communication sleeps
void sleepok(int t, ros::NodeHandle &nh)
{
   if (nh.ok())
                 sleep(t);
}
//** end of function for communication sleeps


// what it should say before detection and following 


// person-detection function
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
    double y_person=msg->people[0].pos.y;
    double x_person=msg->people[0].pos.x;

      // log of position, angle and distance
    double angle_person = (atan2(x_person,-y_person)*180/PI);
    ROS_INFO("Position_Person: %f,%f     Angle_Person: %f Distance_Person:%f", -y_person,x_person, angle_person , ( sqrt(pow(x_person,2)+pow(y_person,2)) ));
      //end of debugging session

      // let robot navigate to (x_robot,y_robot) position to follow human.
      double y_robot = y_person - sin(following_angle)*following_dist;// Y is the left-rigth axis, when left side is positive
      double x_robot = x_person - cos(following_angle)*following_dist; // X is the forward-backward axis, when forward is possitive. theres is a +0.4 because of robot deviation
      ROS_INFO("Going to: %f,%f",-y_robot,x_robot);

     

      // calculate errors in angle and distance
      //Calculate angle error
      double angle_error=atan2(y_robot,x_robot);
      //Calculate distance error
      double distance_error=sqrt(pow(x_robot,2)+pow(y_robot,2));


      
      double linearspeed = (x_robot)*acceleration_coeff;
      
      double angularspeed = (angle_error*angular_vel);

      // sound additions
     


      if (linearspeed>max_speed)
      {
          linearspeed=max_speed;
      }

      if (linearspeed<0){ // means the robot passed the wanted location - so stop
          linearspeed=0;
          
          //const char *str2= "/home/robot/workspace/ros/catkin/Are_you_ok_question.wav";
          //sc.startWave(str2);
          //sleepok(8, nh);
          //sc.stopWave(str2);

      }
      if (linearspeed<0.1 and  abs(90+180*following_angle/PI-angle_person)<10)  angularspeed=0; 

            
      //if (abs(angularspeed)>2*linearspeed and angle_person > 0) angularspeed = 2*angularspeed/(abs(angularspeed))*linearspeed;
     /* if(linearspeed=0){
        if (angle_person>0){
          if (angle_person <90+following_angle) angularspeed = -1;
          else angularspeed =1;
        }
        else{
          if (angle_person>-90) angularspeed = -2;
          else angularspeed = 2;
        }
      }*/
       cmd_vel.linear.x = linearspeed; // move.linear.x = linearspeed
       
       cmd_vel.angular.z = angularspeed; // move.angular.z = angularspeed

      /* suppose to check the error(the distance from wanted place)during following 
      see general thoughts in the end for explanation why it is irelevant
      ROS_INFO("speed: %f, %f",linearspeed,angularspeed);
      if (distance_error>0.2) ROS_INFO("BAD!!!");
      else ROS_INFO("GOOOODDDDD");
      */

	}
  cmd_vel_pub.publish(cmd_vel);  // pub.publish(move)
}


//odometry matters 
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
    ros::NodeHandle nh;
    sound_play::SoundClient sc;

// TODO: find how to allow setting this parameters using launch file
//	 n.param(std::string("kp_distance"), acceleration_coeff, 1.5);
//	 n.param(std::string("distance_target"), following_dist, 1.3);
//	 n.param(std::string("max_speed"), max_speed, 1.2);


// say before subsribing to all others
/*
    while(nh.ok())
  {
  	
 const char *str2= "/home/robot/workspace/ros/catkin/Shalom_intro_Akiva.wav";
        sc.startWave(str2);
        sleepok(8, nh);
        sc.stopWave(str2);
  }
*/
        //sc.playWaveFromPkg("sound_play","/home/robot/workspace/ros/catkin/Are_you_ok_question.wav");
   // other subscriptions
    cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2)); // velocity commands to robot
    
    ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback); // position of the person
    ros::Subscriber sub3 = n.subscribe("/RosAria/pose", 10, poseCallback); // position of the robot  


    // while not rospy.is_shutdown()
    //rate.sleep()

  

    // **soundplay additions
    //sound_play::SoundClient sc;

    
   
        
    ros::spin(); // Keeps looping on the main until interupted from user (using Ctrl-c)  //rospy.spin()

      return 0;
  }
