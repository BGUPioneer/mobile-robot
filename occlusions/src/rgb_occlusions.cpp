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

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
occlusions::sideOcclusions bool_msg;

double AgeThreshold=0;
double ConfidenceTheshold=0.6; //1.1
double HeightTheshold=1.4;

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);  //info about people tracking
    ros::Publisher side= (n.advertise<occlusions::sideOcclusions>("occlusions/sideOcclusions",10));

    double xmin=0;  //from depth image, using the subscriber of people tracking
    double ymin=0;
    double xmax=0;
    double ymax=0;
    double xcenter;
    double personCentroid;
    double distance;
    double confidence;
    double age;
    double height;

    double rgbxmin=0;  //for rgb image
    double rgbymin=0;
    double rgbxmax=0;
    double rgbymax=0;
    double xc;
    double yc;

    bool validTrack;
    int nbOfTracks;
    bool LeftWall= false;
    bool RightWall= false;

public:
    ImageConverter()
      : it_(n)
    {
   //   image_sub = it_.subscribe("/kinect2_head/rgb_rect/image", 10, &ImageConverter::imageCallback, this);
   //   image_sub = it_.subscribe("/kinect2_head/rgb_lowres/image", 10, &ImageConverter::imageCallback, this);
      image_sub = it_.subscribe("/kinect2_head/mono_rect/image", 10, &ImageConverter::imageCallback, this);  //get the mono_rect image (gray image)

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
    xmin=msg->tracks[i].box_2D.x;  //the resolotion of mono_rect is twice the resolotion of depth_rect
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
//  ROS_INFO("rgbxmin: %f", rgbxmin);
//  ROS_INFO("rgbymin: %f", rgbymin);
//  ROS_INFO("rgbxmax: %f", rgbxmax);
//  ROS_INFO("rgbymax: %f", rgbymax);
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
        cv_ptr = cv_bridge::toCvCopy(msg);//now cv_ptr is the matrix of the image

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    xcenter=(xmin+xmax)/2;  //the center of the box in x axis at the DEPTH image

 if(xcenter<270){rgbxmin=round(270-xcenter)+xmin*4;}  //the person is at the left side of the image
 else {rgbxmin=-round(xcenter-270)+xmin*4;}  //the person is at the right side of the image
    rgbxmax=rgbxmin+(xmax-xmin)*2;
    rgbymin=ymin*2;
    rgbymax=rgbymin+(ymax-ymin)*3; //to add more for the legs but not the ground

    xc=(rgbxmin+rgbxmax)/2;  //the center of the box in x axis at the RGB image
    yc=ymin+(rgbymax-rgbymin)/3; //the 1/3 upper body

    int marginAdd= round(10/distance);  //add margin depend on distance
    LeftWall= false; //detect a "tall" vertical occlusion from the left (point of view of the robot)
    RightWall= false; //detect a "tall" vertical occlusion from the left (point of view of the robot)
    int smallOcclusions= round((xc-rgbxmin)/3);  //detect small occlusion
    int bigOcclusions= round((xc-rgbxmin)/2);    //detect big occlusion

          cv::Canny(cv_ptr->image, cv_ptr->image, 50.0, 300.0, 3, false);  //canny edge detector
          cv::Mat element1= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9,9), cv::Point(-1,-1)); // 9*9 open element
          cv::Mat element2= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(-1,-1)); // 5*5 close element
          cv::dilate(cv_ptr->image, cv_ptr->image, element1);  //open the pixels
          cv::erode(cv_ptr->image, cv_ptr->image, element2);  //close the pixels

          cv::Mat temp=cv_ptr->image;
          cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);

              cv::vector<Vec4i> lines;
              HoughLinesP(cv_ptr->image, lines, 1, CV_PI / 180, 50, (rgbymax-rgbymin)/3, 0 );  //find straight lines with Hough

              for (size_t i = 0; i < lines.size(); i++)
              {
                  cv::Vec4i l = lines[i];
                  if (abs(l[0]-l[2])<80) {    //for only vertical lines
                      line(temp, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, 4);
                      //left
                      if (((l[0]>rgbxmin-marginAdd) && (l[0]<xc-5)) || ((l[2]>rgbxmin-marginAdd) && (l[2]<xc-5))){
                          LeftWall=true;
                      }
                      //right
                      if (((l[0]>xc+5) && (l[0]<rgbxmax+marginAdd)) || ((l[2]>xc+5) && (l[2]<rgbxmax+marginAdd))){
                          RightWall=true;
                      }
                  }
                  else {lines.pop_back();}
              }

      // Draw a rectangle around the detected person:
            rectangle(cv_ptr->image,cv::Point(rgbxmin,rgbymin),cv::Point(rgbxmax,rgbymax),cv::Scalar(255,255,255));
            rectangle(temp,cv::Point(rgbxmin,rgbymin),cv::Point(rgbxmax,rgbymax),cv::Scalar(255,255,255));

    // Update GUI Window
          cv::imshow(OPENCV_WINDOW, temp);
          cv::waitKey(3);

          // Output modified video stream
          image_pub.publish(cv_ptr->toImageMsg());

        ROS_INFO("LeftWall: %d", LeftWall);
        ROS_INFO("RightWall: %d", RightWall);

  //publish the boolians variables
  //  bool_msg.bigLeft=bigLeftOcclusions;
  //  bool_msg.smallLeft=smallLeftOcclusions;
    bool_msg.wallLeft=LeftWall;
  //  bool_msg.bigRight=bigRightOcclusions;
  //  bool_msg.smallRight=smallRightOcclusions;
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

