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


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.14159265

cv::Scalar red(0,255,0);

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

double AgeThreshold=0;
double ConfidenceTheshold=0.8; //1.1
double HeightTheshold=1.4;

namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

  //  opt_msgs::DetectionArray opt_[];
 //   opt_msgs::TrackArray opt_[];
  //  ros::Subscriber person_sub = n.subscribe("/detector/detections", 10, &ImageConverter::boxCallback, this);
  //  ros::Subscriber sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::personCallback, this);
    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);


    double xmin=0;
    double ymin=0;
    double xmax=0;
    double ymax=0;
    double personCentroid;
    double distance;
    double confidence;
    double age;
    double height;
    double xc;
    double yc;
    float depth;
    double depthTheshold=200.0;
    bool validTrack;
    int nbOfTracks;
    float temp;



public:
    ImageConverter()
      : it_(n)
    {
      // Subscrive to input video feed and publish output video feed
      image_sub = it_.subscribe("/kinect2_head/depth_rect/image", 10, &ImageConverter::imageCallback, this);
    //  image_sub = it_.subscribe("/kinect2_head/ir_rect_eq/image", 10, &ImageConverter::imageCallback, this);

        //kinect2_head/ir_rect_eq/image
        //kinect2_head/depth_rect/image
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
    xmin=msg->tracks[i].box_2D.x;
    ymin=msg->tracks[i].box_2D.y;
    xmax=xmin+msg->tracks[i].box_2D.width;
    ymax=ymin+msg->tracks[i].box_2D.height;
    distance=msg->tracks[0].distance;
    confidence=msg->tracks[i].confidence;
    height=msg->tracks[i].height;
    age=msg->tracks[i].age;

  //  xc=(xmin+xmax)/2;
 //   yc=ymin+(ymax-ymin)/3; //the 1/3 upper body

 /*   ROS_INFO("xmin: %f", xmin);
    ROS_INFO("ymin: %f", ymin);
    ROS_INFO("xmax: %f", xmax);
    ROS_INFO("ymax: %f", ymax);
    ROS_INFO("Confidence: %f", confidence);
    ROS_INFO("Height: %f", height);
    ROS_INFO("age: %f", age);
    ROS_INFO("distance: %f", distance);
*/
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImagePtr cv_ptr2;
    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr2 is the matrix
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    xc=(xmin+xmax)/2;
    yc=ymin+(ymax-ymin)/3; //the 1/3 upper body
    depth = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.

    ROS_INFO("depth: %f", depth);
  //  ROS_INFO("depth1: %f", depth1);
  //  ROS_INFO("depth2 %f", depth2);

 //   cv::rectangle(cv_ptr->image, cv::Point(xmin, ymin),	cv::Point(xmax, ymax), red, CV_FILLED, 8);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int countLeft=0;
    bool leftOcclusions=false;
 //   if (nbOfTracks>0){
        for (short int i=xmin;i<xc-5;i++){
            for (short int j=ymin;j<ymax-100;j++){
            //   if (cv_ptr->image.at<short int>(cv::Point(i,j))<(depth-depthTheshold)){
     //   depth = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.
        temp=cv_ptr->image.at<short int>(cv::Point(xc+100,yc+100));
                if (temp<(depth-depthTheshold)){
                   countLeft++;
               }
     }
    }
//}
        if (countLeft>1000&&countLeft<2500){
            leftOcclusions=true;
            ROS_INFO("leftOcclusions: little %d", countLeft);
        }
        else  if (countLeft>2500){
            leftOcclusions=true;
            ROS_INFO("leftOcclusions: many %d", countLeft);
        }
        else {ROS_INFO("leftOcclusions: false %d", countLeft);
        ROS_INFO("temp: %f", temp);
        ROS_INFO("depth: %f", depth);
        }



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub.publish(cv_ptr->toImageMsg());

    }
};
int main(int argc, char **argv){


    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::NodeHandle n;

//     ros::Subscriber sub = n.subscribe("/kinect2_head/depth_rect/image", 10, &ImageConverter::imageCallback, &ImageConverter);
//     ros::Subscriber sub1 = n.subscribe("/detector/detections", 10, &ImageConverter::boxCallback, &ImageConverter);

     //kinect2_head/ir_rect_eq/image
//kinect2_head/depth_rect/image
	 ros::spin();
	  return 0;
}

