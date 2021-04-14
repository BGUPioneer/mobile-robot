/*
 * simple_follower.cpp
 *
 *  Created on: May 10, 2017
 *      Author: Hanan Zaichyk
 */



#include "stdio.h"
#include "math.h"
//#include "Vector_follower_Classes.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>
#include <iostream>
#include <list>

using namespace std;

#define PI 3.149265
#define SAMPLES 10 //num of person samples we shall keep = length of the linked list



// class position represent the place of the robot with respect to its orientation 
class RobotPosition
{
public:
  double x,y,orientation;
  RobotPosition();
  RobotPosition(double r_x, double r_y, double r_orientation); // r for right value
  void setRobotPosition(double otherX,double otherY, double otherOrientetion);
  
};


RobotPosition::RobotPosition(): x(0),y(0),orientation(0){}
RobotPosition::RobotPosition(double r_x, double r_y, double r_orientation): x(r_x),y(r_y), orientation(r_orientation){}
void RobotPosition::setRobotPosition(double otherX,double otherY, double otherOrientetion){
  x=otherX;
  y=otherY;
  orientation=otherOrientetion;
}



class Point{
public:
  double x,y;
  Point();
  Point(double xOther, double yOther); // creating new point with given coordinates
  void setPoint(double x, double y);  //  seting the point to new coordinates
  void place(RobotPosition robot);//rotates the point by the orientation of the robot
  double distance(Point other);
};

Point::Point(): x(0),y(0){}
Point::Point(double xOther, double yOther): x(xOther),y(yOther){}
void Point::setPoint (double x, double y){
  this->x = x;
  this->y = y;
}
void Point::place(RobotPosition robot){
  double angle = atan2(x,y)-robot.orientation;
  double dist = sqrt(pow(x,2)+pow(y,2));
  setPoint(dist*sin(angle) + robot.x, dist*cos(angle) + robot.y);
  //ROS_INFO("Robot: %f,%f,%f",robot.x,robot.y,robot.orientation);
  //ROS_INFO("human Position: %f,%f",x,y);
}
double Point::distance(Point other){
  return (sqrt(pow(x-other.x,2)+pow(y-other.y,2)));
}

double MaxLinearSpeed=  0.8;
double MaxAngularSpeed= 0.8;
double KpDistance=      1.0;
double KpAngle=         0.5;
double followingAngle = PI/3;
double DistanceTarget=  0.8;
double PREDICTION =     2.0;  // how much distacne the person going to pass by next iteration compared to last second.

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

//double timepreviousmeasure=0;
//double Xr;
//double Yr;
//double xs;//x position of person in the last iteration. the start of the direction vector
//double ys;//y position of person in the last iteration. the start of the direction vector
double orientation;// orientation of robot. from 0 to 360 derees.
RobotPosition robot;
std::list<Point> points; // list of human detection points in space
double humanSpeed = 0;    // person avg walking speed during last iteration

int samples_counter = 0 ; // count how many human samples we have made.




/////helping functions section

Point calculateTargetPosition(){
  Point start = points.back();
  Point end = points.front();
  double movex = end.x-start.x;
  double movey = end.y-start.y;
  humanSpeed = sqrt(pow(movex,2)+pow(movey,2))/(SAMPLES/10); // samples/10 = the sec pased between start and end samples
  double humanWalkingDirection; // the genaral direction in last SAMPLES/10 seconds
  if(abs(movey)<0.01 and abs(movex)<0.01) {
    humanWalkingDirection = 0;
    PREDICTION=1;
  }

  else{  
    humanWalkingDirection = atan2(end.y-start.y,end.x-start.x);
    PREDICTION=2;
  }
  //prediction point is when the person is going to be by the end of next iteration.
  //basic mode doesnt "Stretch" the human vector and placing the prediction on the last seen point.
  Point prediction = Point(start.x+PREDICTION*(SAMPLES/10)*humanSpeed*cos(humanWalkingDirection),start.y+PREDICTION*(SAMPLES/10)*humanSpeed*sin(humanWalkingDirection));
  //double alpha = atan2(end.y-start.y,end.x-start.x); // the genaral direction in last SAMPLES/10 seconds
 // ROS_INFO("human: %f , %f , walkingdirection : %f",-(end.y-start.y),(end.x-start.x),humanWalkingDirection*180/PI);
  double omega = humanWalkingDirection - PI + followingAngle;
  //debuging
            //ROS_INFO("Start: %f,%f",start.x,start.y);
            //ROS_INFO("End: %f,%f",end.x,end.y);
            //ROS_INFO("Diraction:%f",humanWalkingDirection);
            //ROS_INFO("Calculate: %f,%f",prediction.x,prediction.y);
  //debuging
  
  double goalx = prediction.x-cos(omega)*DistanceTarget;
  double goaly = prediction.y-sin(omega)*DistanceTarget;
  double error = sqrt( pow((robot.x-end.x), 2)+pow((robot.y-end.y), 2) );
  ROS_INFO("Error: %f",error);
  return Point(goalx,goaly);
}


//// end of helping functions section

void personCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  int i=msg->people.size();
 
  if (i>0) {
    Point human = Point(msg->people[0].pos.x,msg->people[0].pos.y);// get the first detected person coordinates relative to robot
    human.place(robot); // place the human's coordinates based on robot position
    points.push_front(human);
    samples_counter = points.size();
    ROS_INFO("");
    if(samples_counter>SAMPLES){ // only whenROS_INFO("start following");        
      points.pop_back();// erase the unneeded old human sample                          
      Point goal = calculateTargetPosition();     
      ROS_INFO("Human speed: %f",humanSpeed);
                                            //ROS_INFO("GOAL: %f,%f",goal.x,goal.y); 
                                            //ROS_INFO("ROBOT: %f,%f",robot.x,robot.y);
                                         
/* TODO: here we should calculate the direction of walk - to deside if person is walking straigh or making a turn.
/double deltaY = yend - ys;
double deltaX = xend - xs;
string move;
ROS_INFO("CHANGE: %f, %f ",deltaY , -deltaX);
if (abs(deltaX) < 0.1 and abs(deltaY) < 0.1) move = "Stop";
else if (abs(deltaY)<0.1) move = "Forward";
else {
  if (abs(deltaX)>abs(deltaY)) move= "Slite Turn ";
  else move = "Sharp Turn ";
  if (deltaY > 0) move = move + "Left";
  else move = move + "Right";
}
ROS_INFO_STREAM (move);*/

      double shiftY = goal.y - robot.y;
      double shiftX = goal.x - robot.x;// SO FAR SO GOOD!! CALCULATE THE RIGHT ANGLE - LEFT TO THE PERSON!
      
      //Calculate angle error
      double AngleError=atan2(shiftY,shiftX)-robot.orientation;
      //Calculate distance error
      double DistanceError=cos(AngleError)*sqrt(pow(shiftX,2)+pow(shiftY,2));////trying. the velocitiy of the robot will be relative only to the distnace it need to go forward not sideways 
                                                          //ROS_INFO("shift:%f,%f,,%f",shiftX,shiftY,atan2(shiftY,shiftX));
                                                          //ROS_INFO("orientation: %f", robot.orientation);

                                                      //    ROS_INFO("Going to: %f,%f", shiftX,shiftY);
      //ROS_INFO("distance to target: %f", DistanceError);

      double angularspeed = AngleError*KpAngle;
      if(abs(angularspeed)>MaxAngularSpeed) angularspeed = angularspeed/abs(angularspeed)*MaxAngularSpeed;
    
     
      double linearspeed=KpDistance*DistanceError;
      if (linearspeed>MaxLinearSpeed) linearspeed=MaxLinearSpeed;
      if (linearspeed<0)  linearspeed=0;
      if (linearspeed<0.1 and  abs(AngleError)<PI/18)  angularspeed=0; // PI/18 ~ 10/180*PI
      
      cmd_vel.linear.x = linearspeed;
      cmd_vel.angular.z = angularspeed;
      //ROS_INFO("SPEED: %f, %f",cmd_vel.linear.x,cmd_vel.angular.z);
     
    }
        
      
  }
   cmd_vel_pub.publish(cmd_vel);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    double orientation = atan2 ( (2*(w*z)), 1-2*pow(z,2));
    robot.setRobotPosition(msg->pose.pose.position.x,msg->pose.pose.position.y,orientation);
    //ROS_INFO("Robot position: %f ",orientation);
    //ROS_INFO("Robot position: %f, %f",Xr , Yr);
    
}

int main(int argc, char **argv){

 
    ros::init(argc, argv, "follower_node");
    ros::NodeHandle n;
    

//   n.param(std::string("kp_distance"), KpDistance, 1.5);
//   n.param(std::string("distance_target"), DistanceTarget, 1.3);
//   n.param(std::string("max_speed"), MaxSpeed, 1.2);
    cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist> ("follower/cmd_vel", 2));
    ros::Subscriber sub = n.subscribe("people_tracker_measurements", 10, personCallback);
    ros::Subscriber sub3 = n.subscribe("/RosAria/pose", 10, poseCallback);   
    ros::spin();

    return 0;
}




/*TODD 5.6.17:
Algorithm is basically working
following fine. all calculations up to calculated goal point is correct

TODO: how to react when need to stop. 
TODO: Alpha changing when it should not.
*/