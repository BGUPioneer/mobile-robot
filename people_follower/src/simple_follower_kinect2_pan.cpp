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

double KpAngle=0.5;
double KpDistance=0.5;
double DistanceTarget=1.2;
double MaxSpeed=0.5;
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
double min=1;
double xp=0;
double yp=0;
double timepreviousmeasure=0;

double AgeThreshold=0;
double ConfidenceTheshold=1.1;
double HeightTheshold=1.4;   //height in meter of the person (minimum)
double HeightMaxTheshold=2.0;   //height in meter of the person (maximum)
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

/*void LaserLegsCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)

{
     xLaserPerson=msg->people.pos.x;
     yLaserPerson=msg->tracks[i].y;
}*/

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
    cmd_vel.angular.z = 0.0;

    //Get the number of tracks in the TrackArray
    int nbOfTracks=msg->tracks.size();

    //If at least 1 track, proceed
    if (nbOfTracks>0) {
        //looping throught the TrackArray
        for(int i=0;i<nbOfTracks && !validTrack;i++){
            //oldest track which is older than the age threshold and above the confidence threshold
            if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold) && (msg->tracks[i].height<HeightMaxTheshold)){
                //Calculate angle error
            //    double AngleError=atan2(msg->tracks[i].y,msg->tracks[i].x);
            //    double AngleError=atan2(msg->tracks[i].y*(sin((AngleErrorPan*180)/ PI)),msg->tracks[i].x*(cos((AngleErrorPan*180)/ PI)));
           //     double xperson=msg->tracks[i].x*(cos((AngleErrorPan*180)/ PI));
           //     double yperson=msg->tracks[i].y*(sin((AngleErrorPan*180)/ PI));

                //Calculate angle error
            //    float xperson=((msg->tracks[i].distance)*cos(AngleSmallError))*(cos(AngleErrorPan));
            //    float yperson=((msg->tracks[i].distance)*cos(AngleSmallError))*(sin(AngleErrorPan));
                float xperson=((msg->tracks[i].distance)*cos(AngleSmallError+AngleErrorPan));
                float yperson=((msg->tracks[i].distance)*sin(AngleSmallError+AngleErrorPan));
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
          //      if(smallError==false ){  //to reduce vibration around 0 angle of the kinect view, and 0 robot angle  // && abs(AngleErrorPan)<0.05
          //    cmd_vel.angular.z = (AngleErrorPan+followingAngle)*KpAngle;
                cmd_vel.angular.z = (AngleError)*KpAngle;
          //      cmd_vel.angular.z = -(AngleError+0.5236)*KpAngle; //30 deg
          //      cmd_vel.angular.z = -(AngleError+1.0472)*KpAngle; //60 deg


             //   cmd_vel.angular.z = AngleError*KpAngle;
            //    }
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


    ros::init(argc, argv, "simple_follower_kinect2_pan");
	 ros::NodeHandle n;

      n.param("/people_follower/Angle", followingAngle, 0.0);

	 cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
     ros::Subscriber sub = n.subscribe("/tracker/tracks", 10, personCallback);
     ros::Subscriber sub2 = n.subscribe("/Pan_Feedback", 10, panCallback);
     ros::Subscriber sub3 = n.subscribe("/Pan_Error_Command", 10, smallErrorCallback);
//     ros::Subscriber sub4 = n.subscribe("/people_tracker_measurements", 10, LaserLegsCallback);
	 ros::spin();
	  return 0;
}













//work
/*void personCallback(const opt_msgs::TrackArray::ConstPtr& msg)
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
        //        double AngleError=atan2(msg->tracks[i].y,msg->tracks[i].x);////////////////////////
         //       double AngleError=atan2(msg->tracks[i].y*sin((panCallback.data*180)/ PI),msg->tracks[i].x*cos((panCallback.data*180)/ PI));///////////////////////

                //Calculate distance error
                double DistanceError=msg->tracks[i].distance-DistanceTarget;
                ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", msg->tracks[i].height);

                //Set command Twist
              //  cmd_vel.angular.z = AngleError*KpAngle;////////////////////////delete
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

void panCallback(const std_msgs::Float32::ConstPtr& msg)
{

    bool validTrack=false;

    //Initialize the twist
 //   cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

                 double AngleError=msg->data;
                //Set command Twist
                 cmd_vel.angular.z = AngleError*KpAngle;///////////////////work
                //Stop for loop
                validTrack=true;

    cmd_vel_pub.publish(cmd_vel);
}


int main(int argc, char **argv){


    ros::init(argc, argv, "simple_follower_kinect2_pan");
     ros::NodeHandle n;

     cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
     ros::Subscriber sub = n.subscribe("/tracker/tracks", 10, personCallback);
     ros::Subscriber sub2 = n.subscribe("/Pan_Feedback", 10, panCallback);
     ros::spin();
      return 0;
}
*/
