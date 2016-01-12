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

#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
#include <occlusions/sideOcclusions.h>
#include <obstacles/laserObstacles.h>

#define PI 3.14159265

//ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

class kinect2_pan_laser
{
    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Subscriber sub6;
    ros::Publisher cmd_vel_pub;

         double KpAngle=0.5;
         double KpDistance=0.3;
         double DistanceTarget=1.2;
         double MaxSpeed=0.8;
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
         double xLaserPerson;
         double yLaserPerson;
         double followingAngle=0;  //15 deg= 0.2618 ,30 deg= 0.5236 rad, 60 deg= 1.0472 rad
         bool kinectLaserMatch=false;
         int nbOfTracksKinect;
         bool BigLeft;
         bool SmallLeft;
         bool WallLeft;
         bool BigRight;
         bool SmallRight;
         bool WallRight;
         bool laser_obstacle_flag;
         bool slow_down_flag;
         double laser_angular_velocity=0;
         double laser_linear_velocity=0;


public:
      kinect2_pan_laser()

      {
         sub1= n.subscribe("/tracker/tracks", 10, &kinect2_pan_laser::personCallback, this);
         sub2= n.subscribe("/Pan_Feedback", 10, &kinect2_pan_laser::panCallback, this);
         sub3= n.subscribe("/Pan_Error_Command", 10, &kinect2_pan_laser::smallErrorCallback, this);
         sub4= n.subscribe("/people_tracker_measurements", 10, &kinect2_pan_laser::LaserLegsCallback, this);
         sub5= n.subscribe("/occlusions/sideOcclusions", 10, &kinect2_pan_laser::occlusionKinectCallback, this);
         sub6= n.subscribe("/obstacles/laserObstacles", 10, &kinect2_pan_laser::LaserObstaclesCallback, this);
         cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
      }

void occlusionKinectCallback(const occlusions::sideOcclusions::ConstPtr& msg)
{
   BigLeft= msg->bigLeft;
   SmallLeft= msg->smallLeft;
   WallLeft= msg->wallLeft;
   BigRight= msg->bigRight;
   SmallRight= msg->smallRight;
   WallRight= msg->wallRight;

   if (BigLeft){followingAngle=-0.5236;}
   else if (SmallLeft){followingAngle=-0.2618;}
   else if (WallLeft){followingAngle=-0.2618;}

   if (BigRight){followingAngle=0.5236;}
   else if (SmallRight){followingAngle=0.2618;}
   else if (WallRight){followingAngle=0.2618;}

   //followingAngle //15 deg= 0.2618 ,30 deg= 0.5236 rad, 60 deg= 1.0472 rad
}

void LaserObstaclesCallback(const obstacles::laserObstacles::ConstPtr& msg)
{
    laser_obstacle_flag=msg->detect_obstacles;
    laser_angular_velocity=msg->angular_velocity;
    laser_linear_velocity=msg->linear_velocity;
    slow_down_flag=msg->slow_down;
    ROS_INFO("laser_obstacle_flag: %d", laser_obstacle_flag);
    ROS_INFO("laser_angular_velocity: %f", laser_angular_velocity);
    if (laser_obstacle_flag){
    cmd_vel.angular.z = laser_angular_velocity;  //turn to avoid obstacles
    cmd_vel.linear.x = laser_linear_velocity;
    cmd_vel_pub.publish(cmd_vel);
    }
}

void smallErrorCallback(const std_msgs::Float32::ConstPtr& msg)
{
     double AngleSmallError=msg->data;
     if ((abs(AngleSmallError)<smallErrorThreshold)&& (abs(AngleErrorPan)<0.01)){smallError=true;}
     else {smallError=false;}
}

void LaserLegsCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

   int nbOfTracksLaser=msg->people.size();

   if (nbOfTracksLaser>0) {
       //Extract coordinates of first detected person
       xLaserPerson=msg->people[0].pos.x;
       yLaserPerson=msg->people[0].pos.y;
       ROS_INFO("xLaser: %f", xLaserPerson);
       ROS_INFO("yLaser: %f", yLaserPerson);

        if (nbOfTracksKinect==0) {
       //Calculate angle error
       double AngleErrorLaser=atan2(yLaserPerson,xLaserPerson);
       //Calculate distance error
       double DistanceErrorLaser=sqrt(pow(xLaserPerson,2)+pow(yLaserPerson,2));

       if(!laser_obstacle_flag){
           cmd_vel.angular.z = AngleErrorLaser*KpAngle;

           double linearspeedLaser=(DistanceErrorLaser-DistanceTarget)*KpDistance;

           if (linearspeedLaser>MaxSpeed)
           {
               linearspeedLaser=MaxSpeed;
           }

           if (linearspeedLaser<0){
               linearspeedLaser=0;
           }
            cmd_vel.linear.x = linearspeedLaser;
           cmd_vel_pub.publish(cmd_vel);
       }
     }

  }
}

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
    nbOfTracksKinect=msg->tracks.size();

    //If at least 1 track, proceed
    if (nbOfTracksKinect>0) {
        //looping throught the TrackArray
        for(int i=0;i<nbOfTracksKinect && !validTrack;i++){
            //oldest track which is older than the age threshold and above the confidence threshold
            if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){

                //Calculate angle error
            //    float xperson=((msg->tracks[i].distance)*cos(AngleSmallError))*(cos(AngleErrorPan));
            //    float yperson=((msg->tracks[i].distance)*cos(AngleSmallError))*(sin(AngleErrorPan));
                float xperson=((msg->tracks[i].distance)*cos(AngleSmallError+AngleErrorPan));
                float yperson=((msg->tracks[i].distance)*sin(AngleSmallError+AngleErrorPan));
                double AngleError=atan2(yperson,xperson);

                double error= sqrt(pow(xperson-xLaserPerson,2)+pow(yperson-yLaserPerson,2));  //calculate the x and y error between the kinect and the laser
                ROS_INFO("error: %f", error);

                if (error<0.2){
                    kinectLaserMatch=true;
                    ROS_INFO("match: %d", kinectLaserMatch);
                }

                //Calculate distance error
                double DistanceError=msg->tracks[i].distance-DistanceTarget;

                //print to the console
/*         ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", msg->tracks[i].height);
                ROS_INFO("distance: %f", msg->tracks[i].distance);
                ROS_INFO("age: %f", msg->tracks[i].age);
                ROS_INFO("AngleError: %f", AngleError);
                ROS_INFO("AngleErrorPan: %f", (AngleErrorPan*180)/ PI);
*/
                ROS_INFO("xperson: %f", xperson);
                ROS_INFO("yperson: %f", yperson);

                //Set command Twist
          //      if(smallError==false ){  //to reduce vibration around 0 angle of the kinect view, and 0 robot angle  // && abs(AngleErrorPan)<0.05
          //    cmd_vel.angular.z = (AngleErrorPan+followingAngle)*KpAngle;
                if (!laser_obstacle_flag){
                cmd_vel.angular.z = (AngleError+followingAngle)*KpAngle;

         //       ROS_INFO("cmd_vel.angular.z: %f", cmd_vel.angular.z);


             //   cmd_vel.angular.z = AngleError*KpAngle;
            //    }
                //Avoid going backward
                if (DistanceError>0.05){  //threshold for small distance error of 0.05 meter
                    double command_speed=DistanceError*KpDistance;
            //        ROS_INFO("VelocityLinear: %f", command_speed);
                    //Limit the speed
                    if (command_speed>MaxSpeed){command_speed=MaxSpeed;}
                    cmd_vel.linear.x = command_speed;
           //         ROS_INFO("VelocityLinear: %f", command_speed);
                }
                //Stop for loop
                validTrack=true;
                cmd_vel_pub.publish(cmd_vel);
             }
        }
    }
  }
}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "simple_follower_kinect2_pan_laser");
    kinect2_pan_laser kpl;
    ros::NodeHandle n;

    ros::spin();
    return 0;
}













