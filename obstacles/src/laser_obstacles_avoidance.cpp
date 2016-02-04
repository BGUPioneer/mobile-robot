
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "rosaria/BumperState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_listener.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "std_srvs/Empty.h"
#include <obstacles/laserObstacles.h>
#include <people_msgs/PositionMeasurementArray.h>
#include "opt_msgs/TrackArray.h"

//geometry_msgs::Twist zero_twist;
obstacles::laserObstacles ob_msg;

class LaserObstacles
{
    ros::NodeHandle n;
    tf::TransformListener tf_listener;

    ros::Subscriber cmdVel = n.subscribe("/cmd_vel", 10, &LaserObstacles::velocityCallback, this);
    ros::Subscriber sub_laser = n.subscribe("/RosAria/S3Series_1_pointcloud", 10, &LaserObstacles::LaserCallback,this);  //get the laser point
    ros::Subscriber sub_people= n.subscribe("/people_tracker_measurements", 10, &LaserObstacles::LaserLegsCallback, this);  //get the leg detector point of the person
    ros::Subscriber sub_people_kinect= n.subscribe("/tracker/tracks", 10, &LaserObstacles::KinectCallback, this);  //get the kinect point of the person
    ros::Subscriber sub2= n.subscribe("/Pan_Feedback", 10, &LaserObstacles::panCallback, this);
    ros::Subscriber sub3= n.subscribe("/Pan_Error_Command", 10, &LaserObstacles::smallErrorCallback, this);


    ros::Publisher pub=(n.advertise<obstacles::laserObstacles> ("/obstacles/laserObstacles",10));

    double DistanceCheck=1.5;  //in front of the robot
    double WidthCheck= 0.5;  //for each side
    double DistanceSlowDownCheck= 1.5;  //for each side
    double angularVelocity;
    double linearVelocity;
    double xLaserPerson;
    double yLaserPerson;
    double xKinectPerson;
    double yKinectPerson;
    double radiusPerson=1.0;
    double AngleErrorPan=0;
    bool smallError=false;
    double smallErrorThreshold=0.01;
    double AngleSmallError=0;


void smallErrorCallback(const std_msgs::Float32::ConstPtr& msg)
{
     double AngleSmallError=msg->data;
     if ((abs(AngleSmallError)<smallErrorThreshold)&& (abs(AngleErrorPan)<0.01)){smallError=true;}
     else {smallError=false;}
}

void panCallback(const std_msgs::Float32::ConstPtr& msg)
{
    AngleErrorPan=msg->data;
}

void LaserLegsCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    int nbOfTracksLaser=msg->people.size();

    if (nbOfTracksLaser>0) {
        for(int i=0; i<nbOfTracksLaser;i++){
        //Extract coordinates of first detected person from leg detector (laser)
        xLaserPerson=msg->people[i].pos.x;
        yLaserPerson=msg->people[i].pos.y;
        ROS_INFO("xLaserPerson :%f", xLaserPerson);
        }
    }
    else{
        xLaserPerson=xKinectPerson;
        yLaserPerson=yKinectPerson;
    }
}

void KinectCallback(const opt_msgs::TrackArray::ConstPtr& msg)
{
    int nbOfTracksKinect=msg->tracks.size();

    if (nbOfTracksKinect>0) {
        for(int i=0; i<nbOfTracksKinect;i++){
        //Extract coordinates of first detected person from the Kinect
        xKinectPerson=((msg->tracks[i].distance)*cos(AngleSmallError+AngleErrorPan));
        yKinectPerson=((msg->tracks[i].distance)*sin(AngleSmallError+AngleErrorPan));
        ROS_INFO("xKinectPerson :%f", xKinectPerson);

        }
    }
}

void LaserCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  sensor_msgs::PointCloud pc_out;
  bool obstacle=false;
  bool SlowDown=false;
  double angularCommand;
  double linearCommand;
  double XclosestObstacle=100.0;
  double YclosestObstacle=100.0;
  double DclosestObstacle=100.0;

  tf_listener.waitForTransform("/laser_frame", (*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
  tf_listener.transformPointCloud("/laser_frame", *msg, pc_out);

  for (int i=0; i<pc_out.points.size() ;i++)
  {
      //an obstacle inside the DistanceCheck and more than radiusPerson from a detected legs or detected person from the Kinect
      if ((pc_out.points[i].x < DistanceSlowDownCheck) && (pc_out.points[i].x >-abs(angularVelocity))  &&
              ( ((sqrt(pow(pc_out.points[i].x-xLaserPerson,2)+pow(pc_out.points[i].y-yLaserPerson,2))>radiusPerson) && (xLaserPerson!=0.0))||
               ((sqrt(pow(pc_out.points[i].x-xKinectPerson,2)+pow(pc_out.points[i].y-yKinectPerson,2))>radiusPerson) && (xKinectPerson!=0.0)) ))
      {
          //2 conditions: 1. y smaller than positive WidthCheck multipile the power of the turn of the robot; 2.y bigger than negative WidthCheck multipile the power of the turn of the robot;
          if ((pc_out.points[i].y < WidthCheck*(1+angularVelocity)) && (pc_out.points[i].y > -WidthCheck*(1-angularVelocity))){
          ROS_INFO("angularVelocity :%f", angularVelocity);
          ROS_INFO("Obstacle detected by laser at : x:%f y:%f", pc_out.points[i].x , pc_out.points[i].y);
          ROS_INFO("laserdistance:%f",(sqrt(pow(pc_out.points[i].x-xLaserPerson,2)+pow(pc_out.points[i].y-yLaserPerson,2))));

            double pointDistance=sqrt(pow(pc_out.points[i].x,2)+pow(pc_out.points[i].y,2));
           //to get the closet obstacle
            if (pointDistance<DclosestObstacle){
                XclosestObstacle=pc_out.points[i].x;
                YclosestObstacle=pc_out.points[i].y;
                DclosestObstacle=pointDistance;
            }
          obstacle=true;
        }
      }
  }
 //   if(XclosestObstacle < DistanceCheck){

        if ((XclosestObstacle < DistanceCheck) && (XclosestObstacle >-abs(angularVelocity))  &&
                ( ((sqrt(pow(XclosestObstacle-xLaserPerson,2)+pow(YclosestObstacle-yLaserPerson,2))>radiusPerson) && (xLaserPerson!=0.0))||
                 ((sqrt(pow(XclosestObstacle-xKinectPerson,2)+pow(YclosestObstacle-yKinectPerson,2))>radiusPerson) && (xKinectPerson!=0.0)) )){

  //    if(linearVelocity>0.2){linearCommand=0.2;}     //move slow near obstacles
  //      else {linearCommand=linearVelocity;}
      linearCommand=0.2;
      //if obstacle from the left than turn right (positive angular velocity)
      if (YclosestObstacle>=0){
      angularCommand=-(WidthCheck-YclosestObstacle)/2;
      }
      //if obstacle from the right than turn left (negative angular velocity)
      else {
          angularCommand=(WidthCheck+YclosestObstacle)/2;
      }
    }else if(linearVelocity>0.3){
        linearCommand=0.3;
        angularCommand=angularVelocity;
        SlowDown=true;
    }else {linearCommand=linearVelocity;}
      //publish the variables
        ob_msg.detect_obstacles=obstacle;
        ob_msg.slow_down=SlowDown;
        ob_msg.angular_velocity= angularCommand;
        ob_msg.linear_velocity= linearCommand;
        pub.publish(ob_msg);
        ROS_INFO("angularCommand :%f", angularCommand);
}


void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    angularVelocity=msg->angular.z;
    linearVelocity=msg->linear.x;
}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_obstacles_avoidance");

  LaserObstacles LO;
  ros::NodeHandle n;

  ros::spin();
  return 0;
}


