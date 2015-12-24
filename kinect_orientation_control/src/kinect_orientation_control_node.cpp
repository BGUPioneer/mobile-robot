#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "opt_msgs/TrackArray.h"
#include "sensor_msgs/JointState.h"
#include <ros/console.h>
#include "math.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "nav_msgs/Odometry.h"


double ConfidenceTheshold=1.1;
double HeightTheshold=1.4;
int TrackedID=0;

double CurrentPosition=0;

double KpAngle=0.2;

ros::Publisher command_pub;
ros::Publisher error_pub;
std_msgs::Float32 error_command;
//ros::Publisher command_trajectory_pub;

ros::ServiceClient setspeed_service;


std_msgs::Float64 pan_command;

ros::Time lastTrackTime;

double polarity=0;
double servo_max_speed=0.5;
double servo_min_speed=0.0001;
double last_speed=0.0001;

//trajectory_msgs::JointTrajectoryPoint traj_point;
//trajectory_msgs::JointTrajectory command_trajectory;

bool TrackInitialized=false;
//SpeedVersion
void personCallback(const opt_msgs::TrackArray::ConstPtr& msg)
{
    bool validTrack=false;
    double AngleError=0;

    //Get the number of tracks in the TrackArray
    int nbOfTracks=msg->tracks.size();

    //If at least 1 track, proceed
    if (nbOfTracks>0) {

        if (!TrackInitialized){
        for(int i=0;i<nbOfTracks;i++){
            if ((msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){
                TrackedID=msg->tracks[i].id;
                TrackInitialized=true;
                ROS_INFO("Found track meeting threshold requirement: %d", TrackedID);
            }
        }
        }
        if (!TrackInitialized){
            ROS_INFO("No valid track found");
        }else
        {
            for(int i=0;i<nbOfTracks && !validTrack;i++){
                if (msg->tracks[i].id==TrackedID){
                    //Calculate angle error
                    AngleError=atan2(msg->tracks[i].y,msg->tracks[i].x);
                    ROS_INFO("Error: %f", AngleError);
                    //Stop for loop
                    validTrack=true;
                    lastTrackTime=ros::Time::now();
                }
            }
        }
    }


    if (validTrack){
          error_command.data=AngleError;
          error_pub.publish(error_command);
    }else if ((ros::Time::now()-lastTrackTime)>ros::Duration(3))
    {
        TrackInitialized=false;
        ROS_INFO("3 sec since last track seen, try to find it");
        }

}


//Position Version
//void personCallback(const opt_msgs::TrackArray::ConstPtr& msg)
//{
//    bool validTrack=false;
//    double AngleError=0;

//        //Get the number of tracks in the TrackArray
//        int nbOfTracks=msg->tracks.size();

//        //If at least 1 track, proceed
//        if (nbOfTracks>0) {
//            //looping throught the TrackArray
//            for(int i=0;i<nbOfTracks && !validTrack;i++){
//                //oldest track which is older than the age threshold and above the confidence threshold
//                if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){
//                    //Calculate angle error
//                    AngleError=CurrentPosition+atan2(msg->tracks[i].y,msg->tracks[i].x);
//                    //ROS_INFO("Error: %f", AngleError);
//                    //Stop for loop
//                    validTrack=true;
//                    }

//                }
//            }

//    if (validTrack){
//
//        double command=AngleError;
//        pan_command.data=command;
//        ROS_INFO("Command: %f", pan_command.data);
//        //pan_command.data=1;
//        command_pub.publish(pan_command);
//    }
//}

int i=0;
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    CurrentPosition=msg->position[0];
    //ROS_INFO("Position: %f", CurrentPosition);
    if (i==0){
//        command_trajectory.points.resize(1);
//        command_trajectory.joint_names.resize(1);
//        command_trajectory.joint_names[0]="pan_joint";

//        traj_point.positions.resize(1);
//        traj_point.velocities.resize(1);
//        traj_point.accelerations.resize(1);
//        traj_point.effort.resize(1);

//        traj_point.positions[0]=1.5;
//        traj_point.velocities[0]=0;


//        command_trajectory.points[0]=traj_point;

//        command_trajectory_pub.publish(command_trajectory);


        i++;
    }
}

//void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//    double desired_pan_speed=-(msg->twist.twist.angular.z);

//    double current_polarity=copysign(1, desired_pan_speed);
//    if (current_polarity!=polarity){
//        polarity=current_polarity;
//        pan_command.data=polarity*3.1;
//        command_pub.publish(pan_command);
//    }

//    double speed=desired_pan_speed;
//    if (speed!=last_speed) {
//    setspeed_msg.request.speed=fabs(speed);
//    last_speed=speed;
//    if (!setspeed_service.call(setspeed_msg)){
//        ROS_INFO("Service call error");
//    }
//    ROS_INFO("Speed command: %f", setspeed_msg.request.speed);
//    }


//}

int main(int argc, char **argv){


     ros::init(argc, argv, "orientation_control");
     ros::NodeHandle n;

     lastTrackTime= ros::Time::now();

//     ros::service::waitForService("/pan_joint/set_speed");
//     ROS_INFO("Service Ready");
//     setspeed_service = n.serviceClient<arbotix_msgs::SetSpeed>("/pan_joint/set_speed",true);

     //command_pub = n.advertise<std_msgs::Float64>("/pan_joint/command", 2);
     error_pub = n.advertise<std_msgs::Float32>("/Pan_Error_Command", 1);
     //command_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/pan_controller/command", 1);
     ros::Subscriber sub_person = n.subscribe("/tracker/tracks", 1, personCallback);
     //ros::Subscriber sub_joint = n.subscribe("/joint_states", 10, jointCallback);
     //ros::Subscriber sub_pose= n.subscribe("/RosAria/pose", 10, poseCallback);

     ros::spin();
     return 0;
}
