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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opt_msgs/DetectionArray.h"
#include <occlusions/sideOcclusions.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.14159265

cv::Scalar red(0,255,0);

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
occlusions::sideOcclusions bool_msg;

double AgeThreshold=0;
double ConfidenceTheshold=0.6; //1.1
double HeightTheshold=1.4;

namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);
    ros::Publisher side= (n.advertise<occlusions::sideOcclusions>("occlusions/sideOcclusions",10));

    double xmin=0;
    double ymin=0;
    double xmax=0;
    double ymax=0;
    double xcenter;
    double rgbxmin=0;
    double rgbymin=0;
    double rgbxmax=0;
    double rgbymax=0;
    double personCentroid;
    double distance;
    double confidence;
    double age;
    double height;
    double xc;
    double yc;
    float depth;
    double depthTheshold=800.0;
    bool validTrack;
    int nbOfTracks;
    float temp;
    float value;
    cv::Mat roi;

public:
    ImageConverter()
      : it_(n)
    {
   //   image_sub = it_.subscribe("/kinect2_head/rgb_rect/image", 10, &ImageConverter::imageCallback, this);
   //   image_sub = it_.subscribe("/kinect2_head/rgb_lowres/image", 10, &ImageConverter::imageCallback, this);
      image_sub = it_.subscribe("/kinect2_head/mono_rect/image", 10, &ImageConverter::imageCallback, this);

      image_pub = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }


void boxCallback(const opt_msgs::TrackArray::ConstPtr& msg){

    bool validTrack=false;

    int nbOfTracks=msg->tracks.size();
     if (nbOfTracks>0) {
         for(int i=0;i<nbOfTracks && !validTrack;i++){
             //oldest track which is older than the age threshold and above the confidence threshold
             if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){
    xmin=msg->tracks[i].box_2D.x;  //the resolotion of rgb_rect is twice the resolotion of depth_rect
    ymin=msg->tracks[i].box_2D.y;
    xmax=xmin+msg->tracks[i].box_2D.width;
    ymax=ymin+msg->tracks[i].box_2D.height;
    distance=msg->tracks[i].distance;
    confidence=msg->tracks[i].confidence;
    height=msg->tracks[i].height;
    age=msg->tracks[i].age;

//    ROS_INFO("xmin: %f", xmin);
//    ROS_INFO("ymin: %f", ymin);
//    ROS_INFO("xmax: %f", xmax);
//    ROS_INFO("ymax: %f", ymax);
  ROS_INFO("rgbxmin: %f", rgbxmin);
  ROS_INFO("rgbymin: %f", rgbymin);
  ROS_INFO("rgbxmax: %f", rgbxmax);
  ROS_INFO("rgbymax: %f", rgbymax);
//    ROS_INFO("Confidence: %f", confidence);
//    ROS_INFO("Height: %f", height);
//    ROS_INFO("age: %f", age);
//    ROS_INFO("distance: %f", distance);

validTrack=true;

            }
        }
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
 //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix
        cv_ptr = cv_bridge::toCvCopy(msg);//now cv_ptr is the matrix

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

 //   cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);   //convert to gray

/*    rgbxmin=xmin*3.5;
    rgbymin=ymin;
    rgbxmax=xmax*4;
    rgbymax=ymax*2;
*/
    xcenter=(xmin+xmax)/2;
 //   rgbxmin=round((270-xcenter)/10)*round(distance/2)+xmin*2;   //for rgb_lowers
 //   rgbxmax=-round((270-xcenter)/10)*round(distance/2)+xmax*2;
 if (xcenter<270){   rgbxmin=round((270-xcenter)/2)+xmin*4; }
 else {rgbxmin=-round((xcenter-270)/2)+xmin*4; }
    rgbxmax=rgbxmin+(xmax-xmin)*2;
    ROS_INFO("xcenter: %f", xcenter);


    rgbymin=ymin*2;
    rgbymax=ymax*2;




/*    roi=cv::Mat2d(rgbxmax-rgbxmin,rgbymax-rgbymin);
    short int tx=rgbxmin;
    short int ty=rgbymin;
    for (int i=0;i<rgbxmax-rgbxmin-1;i++){
        for (int j=0;j<rgbymax-rgbymin-1;j++){
            roi.at<int>(i,j)=cv_ptr->image.at<short int>(cv::Point(tx,ty));
            tx++;
            ty++;
        }
    }
*/
    // Draw a rectangle around the detected person:
          rectangle(cv_ptr->image,cv::Point(rgbxmin,rgbymin),cv::Point(rgbxmax,rgbymax),cv::Scalar(255,255,255));
   //       rectangle(cv_ptr->image,cv::Point(xmin*2,ymin),cv::Point(xmax*2,ymax),cv::Scalar(255,255,255));


    // Update GUI Window
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
          cv::waitKey(3);

          // Output modified video stream
          image_pub.publish(cv_ptr->toImageMsg());


    xc=(rgbxmin+rgbxmax)/2;
    yc=ymin+(rgbymax-rgbymin)/3; //the 1/3 upper body
    value = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.


    ROS_INFO("value: %f", value);
  //  ROS_INFO("depth2 %f", depth2);

//left and right detections
    int downCut= round((ymax-ymin)/8);  //to cut the lower part of the person for reduce floor alarm
    int smallOcclusions= round(((xc-xmin)/3)*(ymax-ymin)*7/8);  //detect small occlusion
    int bigOcclusions= round(((xc-xmin)/2)*(ymax-ymin)*7/8);    //detect big occlusion
    int marginAdd= round(10/distance);  //add margin depend on distance

    // size of a pixel depend on depth
   // x = (X - 3.3931e+02) * z / 5.9421e+02
   // y = (Y - 2.4274e+02) * z / 5.9421e+02

    int countLeft= 0;
    int countRight= 0;

    bool smallLeftOcclusions= false; //detect occlusion from the left (point of view of the robot)
    bool bigLeftOcclusions= false; //detect occlusion from the left (point of view of the robot)
    bool LeftWall= false; //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
    int countLeftWall= 0;

    bool smallRightOcclusions= false; //detect occlusion from the left (point of view of the robot)
    bool bigRightOcclusions= false; //detect occlusion from the left (point of view of the robot)
    bool RightWall= false; //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
    int countRightWall= 0;

    if ((age>AgeThreshold) && (confidence>ConfidenceTheshold) && (height>HeightTheshold)){

   //left
        for (short int i=xmin-marginAdd;i<xc-5;i++){
            countLeftWall= 0;
            for (short int j=ymin;j<ymax-downCut;j++){
        temp=cv_ptr->image.at<short int>(cv::Point(i,j));
                if (temp<(depth-depthTheshold)){
                   countLeft++;
                   countLeftWall++;
                   if (countLeftWall==ymax-downCut-ymin){LeftWall=true;}
               }
        }
    }

   //right
        for (short int i=xc+5;i<xmax+marginAdd;i++){
            countRightWall= 0;
            for (short int j=ymin;j<ymax-downCut;j++){
        temp=cv_ptr->image.at<short int>(cv::Point(i,j));
                if (temp<(depth-depthTheshold)){
                   countRight++;
                   countRightWall++;
                   if (countRightWall==ymax-downCut-ymin){RightWall=true;}
               }
        }
    }

 //       ROS_INFO("LeftWall: %d", LeftWall);
 //       ROS_INFO("depth-depthTheshold: %f", depth-depthTheshold);


        if (countLeft>smallOcclusions&&countLeft<bigOcclusions){
            smallLeftOcclusions=true;
  //          ROS_INFO("LeftOcclusions: small %d", countLeft);
        }
        else  if (countLeft>bigOcclusions){
            bigLeftOcclusions=true;
 //           ROS_INFO("LeftOcclusions: big %d", countLeft);
        }
        else {
  //          ROS_INFO("LeftOcclusions: false %d", countLeft);
 //       ROS_INFO("depth: %f", depth);
        }

  //      ROS_INFO("RightWall: %d", RightWall);
  //      ROS_INFO("LeftWall: %d", LeftWall);

        if (countRight>smallOcclusions&&countRight<bigOcclusions){
            smallRightOcclusions=true;
  //          ROS_INFO("RightOcclusions: small %d", countRight);
        }
        else  if (countRight>bigOcclusions){
            bigRightOcclusions=true;
  //          ROS_INFO("RightOcclusions: big %d", countRight);
        }
        else {
  //          ROS_INFO("RightOcclusions: false %d", countRight);
  //      ROS_INFO("depth: %f", depth);
        }
}

  //publish the boolians variables
    bool_msg.bigLeft=bigLeftOcclusions;
    bool_msg.smallLeft=smallLeftOcclusions;
    bool_msg.wallLeft=LeftWall;
    bool_msg.bigRight=bigRightOcclusions;
    bool_msg.smallRight=smallRightOcclusions;
    bool_msg.wallRight=RightWall;

    side.publish(bool_msg);

    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::NodeHandle n;

	 ros::spin();
	  return 0;
}

