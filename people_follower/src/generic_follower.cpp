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

double KpAngle=0.5;
double KpDistance=0.5;
double DistanceTarget=1.2;
double MaxSpeed=1.2;
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
double min=1;
double xp=0;
double yp=0;
double timepreviousmeasure=0;

double AgeThreshold=0;
double ConfidenceTheshold=1.1;
double HeightTheshold=1.4;
double AngleErrorPan=0;
bool smallError=false;
double smallErrorThreshold=0.01;
double AngleSmallError=0;
double xLaserPerson=0;
double yLaserPerson=0;


void smallErrorCallback(const std_msgs::Float32::ConstPtr& msg)
{
     double AngleSmallError=msg->data;
     if ((abs(AngleSmallError)<smallErrorThreshold)&& (abs(AngleErrorPan)<0.01)){smallError=true;}
     else {smallError=false;}
}


double followingAngle; //30 deg= 0.5236 rad, 60 deg= 1.0472 rad
//double followingAngle= 0.5236;  //30 deg
//double followingAngle= 1.0472;  //60 deg

void panCallback(const std_msgs::Float32::ConstPtr& msg)
{
    AngleErrorPan=msg->data;
}

void personCallback(const opt_msgs::TrackArray::ConstPtr& msg)
{
    bool validTrack=false;

    //Initialize the twist
  	cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
  	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

   
                float xperson=();
                float yperson=();
                double AngleError=atan2(yperson,xperson);


                //Calculate distance error
                double DistanceError=msg->tracks[i].distance-DistanceTarget;

                //print to the console
                ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", msg->tracks[i].height);
                ROS_INFO("distance: %f", msg->tracks[i].distance);
                ROS_INFO("age: %f", msg->tracks[i].age);
                ROS_INFO("AngleError: %f", AngleError);
                ROS_INFO("AngleErrorPan: %f", (AngleErrorPan*180)/ PI);
                ROS_INFO("xperson: %f", xperson);
                ROS_INFO("yperson: %f", yperson);

                //Set command Twist

                cmd_vel.angular.z = -(AngleError)*KpAngle;

                //Avoid going backward
                if (DistanceError>0.05){  //threshold for small distance error of 0.05 meter
                    double command_speed=DistanceError*KpDistance;
                    ROS_INFO("VelocityLinear: %f", command_speed);
                    //Limit the speed
                    if (command_speed>MaxSpeed){command_speed=MaxSpeed;}
                    cmd_vel.linear.x = command_speed;
                    ROS_INFO("VelocityLinear: %f", command_speed);
                }
                //Stop for loop
                validTrack=true;
            }
            else{
                ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", 0.0);
                ROS_INFO("distance: %f", 0.0);
                ROS_INFO("age: %f", 0.0);
                ROS_INFO("AngleError: %f", 90.0);
                ROS_INFO("AngleErrorPan: %f", 180.0);
                ROS_INFO("xperson: %f", 0.0);
                ROS_INFO("yperson: %f", 0.0);
            }
        }
    }
    cmd_vel_pub.publish(cmd_vel);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "generic_follower");
	ros::NodeHandle n;
	n.param("/people_follower/Angle", followingAngle, 0.0);

	//subscribers	
	ros::Subscriber sub = n.subscribe("/tracker/tracks", 10, personCallback);// create new topic
	
	//publishers
	cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
	     
	 
	ros::spin();
	return 0;
}





