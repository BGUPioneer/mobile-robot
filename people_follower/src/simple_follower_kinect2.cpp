#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "opt_msgs/TrackArray.h"
#include <ros/console.h>

#define PI 3.14159265

double KpAngle=2;
double KpDistance=1.2;
double DistanceTarget=1.2;
double MaxSpeed=0.8;
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
double min=1;
double xp=0;
double yp=0;
double timepreviousmeasure=0;

double AgeThreshold=0;
double ConfidenceTheshold=1.1;
double HeightTheshold=1.5;

void personCallback(const opt_msgs::TrackArray::ConstPtr& msg)
{

    bool validTrack=false;

    //Initialize the twist
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    //Get the number of tracks in the TrackArray
    int nbOfTracks=msg->tracks.size();

    //If at least 1 track, proceed
    if (nbOfTracks>0) {
        //looping throught the TrackArray
        for(int i=0;i<nbOfTracks && !validTrack;i++){
            //oldest track which is older than the age threshold and above the confidence threshold
            if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){
                //Calculate angle error
                double AngleError=atan2(msg->tracks[i].y,msg->tracks[i].x);
                //Calculate distance error
                double DistanceError=msg->tracks[i].distance-DistanceTarget;
                ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", msg->tracks[i].height);

                //Set command Twist
                cmd_vel.angular.z = AngleError*KpAngle;
                //Avoid going backward
                if (DistanceError>0){
                    double command_speed=DistanceError*KpDistance;
                    //Limit the speed
                    if (command_speed>MaxSpeed){command_speed=MaxSpeed;}
                    cmd_vel.linear.x = command_speed;
                }
                //Stop for loop
                validTrack=true;
            }
        }
    }
    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){


    ros::init(argc, argv, "simple_follower_kinect2");
	 ros::NodeHandle n;

	 cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
     ros::Subscriber sub = n.subscribe("/tracker/tracks", 10, personCallback);
	 ros::spin();
	  return 0;
}


