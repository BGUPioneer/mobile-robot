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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
//    ros::Subscriber sub7;
    ros::Subscriber sub8;

    ros::Publisher vis_pub1;
    ros::Publisher vis_pub2;
    ros::Publisher vis_pub3;
    ros::Publisher cmd_vel_pub;

         double KpAngle=0.5;
         double KpDistance=0.2;
         double DistanceTarget=1.2;
         double MaxSpeed=0.5;
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
         double linearspeedLaser;
         double DistanceErrorLaser;
         double DistanceErrorKinect;
         double linearspeedKinect;
         double xRobot;
         double yRobot;
         double orientationRobot;
         double xperson;
         double yperson;
         double AngleError;
         double xPath;
         double yPath;
         double distanceKinect;
         double xFollow;
         double yFollow;
         double AngleErrorFollow;
         double DistanceErrorFollow;
         double followingAngle=0;  //15 deg= 0.2618 ,30 deg= 0.5236 rad, 60 deg= 1.0472 rad
         bool kinectLaserMatch=false;
         int nbOfTracksKinect=0;
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
         double AngleErrorLaser;
         double AngleErrorKinect;
         double error;
         double age;
         double height;
         double confidence;
         std::vector<double> XpathPoints;
         std::vector<double> YpathPoints;

public:
      kinect2_pan_laser()

      {
         sub1= n.subscribe("/tracker/tracks", 10, &kinect2_pan_laser::personCallback, this);                      //the kinect parameters of the person
         sub2= n.subscribe("/Pan_Feedback", 10, &kinect2_pan_laser::panCallback, this);                           //the angle of the pan from the center of the robot
         sub3= n.subscribe("/Pan_Error_Command", 10, &kinect2_pan_laser::smallErrorCallback, this);               //the angle of the person from the center of the kinect
         sub4= n.subscribe("/people_tracker_measurements", 10, &kinect2_pan_laser::LaserLegsCallback, this);      //the laser leg detector parameters of the person
         sub5= n.subscribe("/occlusions/sideOcclusions", 10, &kinect2_pan_laser::occlusionKinectCallback, this);  //occlusions from depth or rgb
         sub6= n.subscribe("/obstacles/laserObstacles", 10, &kinect2_pan_laser::LaserObstaclesCallback, this);    //obstacles from laser
 //        sub7= n.subscribe("/tracker/history", 10, &kinect2_pan_laser::historyTrackCallback, this);
         sub8= n.subscribe("/RosAria/pose", 10, &kinect2_pan_laser::poseCallback, this);                          //position of the robot in the world
         vis_pub1 = ros::Publisher(n.advertise<visualization_msgs::Marker>( "/visualization_marker_array", 1 ));  //for laser legs (green)
         vis_pub2 = ros::Publisher(n.advertise<visualization_msgs::Marker>( "/visualization_marker_array", 1 ));  //for Kinect person detected (blue)
         vis_pub3 = ros::Publisher(n.advertise<visualization_msgs::Marker>( "/visualization_marker_array", 1 ));  //for robot position (red), when using rviz set the fixed frame to odom

         cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));


      }



void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  xRobot=msg->pose.pose.position.x;
  yRobot=msg->pose.pose.position.y;
//  orientationRobot=msg->pose.pose.orientation.z;
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  orientationRobot= tf::getYaw(pose.getRotation());  //get radiand rotation (0 front, 3.14 back, left positive, right negative)
//  ROS_INFO("xrobot: %f", xRobot);
//  ROS_INFO("yrobot: %f", yRobot);
//  ROS_INFO("orientationRobot: %f", PI-orientationRobot);


  ROS_INFO("xRobot: %f", xRobot);
  ROS_INFO("yRobot: %f", yRobot);
  ROS_INFO("orientationRobot: %f", orientationRobot);
  ROS_INFO("BigLeft: %d", BigLeft);
  ROS_INFO("SmallLeft: %d", SmallLeft);
  ROS_INFO("WallLeft: %d", WallLeft);
  ROS_INFO("BigRight: %d", BigRight);
  ROS_INFO("SmallRight: %d", SmallRight);
  ROS_INFO("WallRight: %d", WallRight);
  ROS_INFO("laser_obstacle_flag: %d", laser_obstacle_flag);
  ROS_INFO("laser_angular_velocity: %f", laser_angular_velocity);
  ROS_INFO("xLaser: %f", xLaserPerson);
  ROS_INFO("yLaser: %f", yLaserPerson);
  ROS_INFO("AngleErrorLaser: %f", AngleErrorLaser);
  ROS_INFO("KinectLaserError: %f", error);
  ROS_INFO("match: %d", kinectLaserMatch);
  ROS_INFO("Confidence: %f", confidence);
  ROS_INFO("Height: %f", height);
  ROS_INFO("distanceKinect: %f", distanceKinect);
  ROS_INFO("age: %f", age);
  ROS_INFO("AngleErrorKinect: %f", AngleErrorKinect);
  ROS_INFO("AngleErrorPan: %f", (AngleErrorPan*180)/ PI);
  ROS_INFO("xKinect: %f", xperson);
  ROS_INFO("yKinect: %f", yperson);
  ROS_INFO("AngleErrorFollow: %f", AngleErrorFollow);
  ROS_INFO("xPath: %f", xPath);
  ROS_INFO("yPath: %f", yPath);
  ROS_INFO("xFollow: %f", xFollow);
  ROS_INFO("yFollow: %f", yFollow);



  //////////////////////////////////marker
          visualization_msgs::Marker marker;
          marker.header.frame_id = "odom";
          marker.header.stamp = ros::Time();
          marker.ns = "robotPose";
          marker.id = 0;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = xRobot;
          marker.pose.position.y = yRobot;
          marker.pose.position.z = 0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
       //   marker.pose.orientation.w = orientationRobot;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.3;
          marker.scale.y = 0.3;
          marker.scale.z = 0.1;
          marker.color.a = 1.0; // Don't forget to set the alpha!
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          vis_pub3.publish( marker );
  ////////////////////////
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
//    ROS_INFO("laser_obstacle_flag: %d", laser_obstacle_flag);
//    ROS_INFO("laser_angular_velocity: %f", laser_angular_velocity);
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
//       ROS_INFO("xLaser: %f", xLaserPerson);
//       ROS_INFO("yLaser: %f", yLaserPerson);
//       ROS_INFO("nbOfTracksKinect: %d", nbOfTracksKinect);


        if (nbOfTracksKinect==0) {
       //Calculate angle error
       AngleErrorLaser=atan2(yLaserPerson,xLaserPerson);
       //Calculate distance error
       DistanceErrorLaser=sqrt(pow(xLaserPerson,2)+pow(yLaserPerson,2));

       xPath= xRobot+cos(orientationRobot+AngleErrorLaser)*DistanceErrorLaser;
       yPath= yRobot+sin(orientationRobot+AngleErrorLaser)*DistanceErrorLaser;
//       ROS_INFO("xPath: %f", xPath);
//       ROS_INFO("yPath: %f", yPath);


       XpathPoints.insert(XpathPoints.begin(),xPath);
       YpathPoints.insert(YpathPoints.begin(),yPath);
//       ROS_INFO("size: %d", XpathPoints.size());

       if (XpathPoints.size()>31){
       xFollow=XpathPoints.at(30);
       yFollow=YpathPoints.at(30);

//       ROS_INFO("xFollow: %f", xFollow);
//       ROS_INFO("yFollow: %f", yFollow);


       XpathPoints.pop_back();
       YpathPoints.pop_back();


       if(!laser_obstacle_flag){
           AngleErrorFollow=PI-atan2(yFollow-yRobot,(xFollow-xRobot))-PI+orientationRobot;
           if(abs(AngleErrorFollow)>PI){
               if(AngleErrorFollow<0){AngleErrorFollow=AngleErrorFollow+2*PI;}
               else{AngleErrorFollow=AngleErrorFollow-2*PI;}
           }

//           ROS_INFO("angle: %f", PI-atan2(yFollow-yRobot,(xFollow-xRobot)));
//           ROS_INFO("AngleErrorFollow: %f", AngleErrorFollow);


       cmd_vel.angular.z =-AngleErrorFollow*KpAngle;

       DistanceErrorFollow=sqrt(pow(xFollow-xRobot,2)+pow(yFollow-yRobot,2));
       if (DistanceErrorLaser>DistanceTarget){ linearspeedLaser=(DistanceErrorFollow)*KpDistance;}
       else{linearspeedLaser=0;}
//       ROS_INFO("DistanceErrorLaser: %f", DistanceErrorLaser);

       if (linearspeedLaser>MaxSpeed)
       {
           linearspeedLaser=MaxSpeed;
       }

       if (linearspeedLaser<0){
           linearspeedLaser=0;
       }
        cmd_vel.linear.x = linearspeedLaser;

       }
      }
     }
        //////////////////////////////////marker
                visualization_msgs::Marker marker;
                marker.header.frame_id = "base_link";
                marker.header.stamp = ros::Time();
                marker.ns = "laser";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = xFollow;
                marker.pose.position.y = yFollow;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                vis_pub1.publish( marker );
        ////////////////////////
    }

   cmd_vel_pub.publish(cmd_vel);
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
                distanceKinect=msg->tracks[i].distance;
                xperson=((distanceKinect)*cos(AngleSmallError+AngleErrorPan));
                yperson=((distanceKinect)*sin(AngleSmallError+AngleErrorPan));
                AngleErrorKinect=atan2(yperson,xperson);
                age=msg->tracks[i].age;
                height=msg->tracks[i].height;
                confidence=msg->tracks[i].confidence;


           //     xPath= xRobot-cos(PI-orientationRobot+AngleSmallError+AngleErrorPan)*distanceKinect;
           //     yPath= yRobot-sin(PI-orientationRobot+AngleSmallError+AngleErrorPan)*distanceKinect;
                xPath= xRobot+cos(orientationRobot+AngleErrorKinect)*distanceKinect;
                yPath= yRobot+sin(orientationRobot+AngleErrorKinect)*distanceKinect;
           //     xPath= xRobot+cos(orientationRobot+AngleSmallError+AngleErrorPan)*distanceKinect;
           //     yPath= yRobot+sin(orientationRobot+AngleSmallError+AngleErrorPan)*distanceKinect;
//                ROS_INFO("xPath: %f", xPath);
//                ROS_INFO("yPath: %f", yPath);

                XpathPoints.insert(XpathPoints.begin(),xPath);
                YpathPoints.insert(YpathPoints.begin(),yPath);
          //      XpathPoints.insert(XpathPoints.begin(),xperson);
          //      YpathPoints.insert(YpathPoints.begin(),yperson);
//                ROS_INFO("size: %d", XpathPoints.size());

                if (XpathPoints.size()>91){
                xFollow=XpathPoints.at(90);
                yFollow=YpathPoints.at(90);

//                ROS_INFO("xFollow: %f", xFollow);
//                ROS_INFO("yFollow: %f", yFollow);


                XpathPoints.pop_back();
                YpathPoints.pop_back();

                error= sqrt(pow(xperson-xLaserPerson,2)+pow(yperson-yLaserPerson,2));  //calculate the x and y error between the kinect and the laser
//                ROS_INFO("error: %f", error);

                if (error<0.2){
                    kinectLaserMatch=true;
//                    ROS_INFO("match: %d", kinectLaserMatch);
                }else{kinectLaserMatch=false;}

                //Calculate distance error
                DistanceErrorKinect=msg->tracks[i].distance;

                //print to the console
/*         ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", msg->tracks[i].height);
                ROS_INFO("distance: %f", msg->tracks[i].distance);
                ROS_INFO("age: %f", msg->tracks[i].age);
                ROS_INFO("AngleError: %f", AngleError);
                ROS_INFO("AngleErrorPan: %f", (AngleErrorPan*180)/ PI);
*/
//                ROS_INFO("xperson: %f", xperson);
//                ROS_INFO("yperson: %f", yperson);

                //Set command Twist
          //      if(smallError==false ){  //to reduce vibration around 0 angle of the kinect view, and 0 robot angle  // && abs(AngleErrorPan)<0.05
          //    cmd_vel.angular.z = (AngleErrorPan+followingAngle)*KpAngle;


                if (!laser_obstacle_flag){
                    AngleErrorFollow=PI-atan2(yFollow-yRobot,(xFollow-xRobot))-PI+orientationRobot;
                //    AngleErrorFollow=atan2(yFollow-yRobot,(xFollow-xRobot));
                //    AngleErrorFollow=atan2(yFollow,xFollow);


                    if(abs(AngleErrorFollow)>PI){
                        if(AngleErrorFollow<0){AngleErrorFollow=AngleErrorFollow+2*PI;}
                        else{AngleErrorFollow=AngleErrorFollow-2*PI;}
                    }

//                    ROS_INFO("angle: %f", PI-atan2(yFollow-yRobot,(xFollow-xRobot)));
//                    ROS_INFO("AngleErrorFollow: %f", AngleErrorFollow);


                cmd_vel.angular.z =-AngleErrorFollow*KpAngle;

                DistanceErrorFollow=sqrt(pow(xFollow-xRobot,2)+pow(yFollow-yRobot,2));
                if (DistanceErrorKinect>DistanceTarget){ linearspeedKinect=(DistanceErrorFollow)*KpDistance;}
                else{linearspeedKinect=0;}
//                ROS_INFO("DistanceErrorLaser: %f", DistanceErrorLaser);

                if (linearspeedKinect>MaxSpeed)
                {
                    linearspeedKinect=MaxSpeed;
                }

                if (linearspeedKinect<0 || DistanceErrorKinect<0.05 ){
                    linearspeedKinect=0;
                }
                 cmd_vel.linear.x = linearspeedKinect;
                 //Stop for loop
                 validTrack=true;
                 cmd_vel_pub.publish(cmd_vel);
                }

                }


            }
            else{
 /*               ROS_INFO("Confidence: %f", msg->tracks[i].confidence);
                ROS_INFO("Height: %f", 0.0);
                ROS_INFO("distance: %f", 0.0);
                ROS_INFO("age: %f", 0.0);
                ROS_INFO("AngleError: %f", 90.0);
                ROS_INFO("AngleErrorPan: %f", 180.0);
                ROS_INFO("xperson: %f", 0.0);
                ROS_INFO("yperson: %f", 0.0);
 */
            }
            //////////////////////////////////marker
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "base_link";
                    marker.header.stamp = ros::Time();
                    marker.ns = "kinect";
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = xFollow;
                    marker.pose.position.y = yFollow;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                    vis_pub2.publish( marker );
            ////////////////////////
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













