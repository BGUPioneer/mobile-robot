/* 

this node gets detections from the leg detector and pass it on to the follower.

*/

#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
#include "geometry_msgs/Point.h"
#include <ros/console.h>

#define PI 3.14159265

ros::Publisher person_xy_pub;
geometry_msgs::Point personXY;


void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
  int i=msg->people.size();

  if (i>0) {

	//Extract coordinates of first detected person
	personXY.x=msg->people[0].pos.x;
	personXY.y=msg->people[0].pos.y;
      
	//publish
	person_xy_pub.publish(cmd_vel);
	}
	else {
		ROS_INFO("no detection");	
	}
}

int main(int argc, char **argv){

    ros::init(argc, argv, "laser_person_detector");
    ros::NodeHandle n;

	person_xy_pub = ros::Publisher(n.advertise<geometry_msgs::Point> ("follower/person_xy", 2));// change to new topic

	ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback);

	ros::spin();

      return 0;
}




