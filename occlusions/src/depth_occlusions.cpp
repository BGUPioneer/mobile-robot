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


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    opt_msgs::DetectionArray opt_[];
  //  opt_msgs::Subscriber person_sub;
    ros::Subscriber person_sub = n.subscribe("/detector/detections", 10, &ImageConverter::boxCallback, this);

  //  Subscriber person_sub = n.subscribe("/detector/detections", 10, &ImageConverter::boxCallback, this);
    double xmin;
    double ymin;
    double xmax;
    double ymax;
    double personCentroid;
    double xcentroid;
    double ycentroid;

public:
    ImageConverter()
      : it_(n)
    {
      // Subscrive to input video feed and publish output video feed
    //  image_sub = it_.subscribe("/kinect2_head/depth_rect/image", 10, &ImageConverter::imageCallback, this);
      image_sub = it_.subscribe("/kinect2_head/ir_rect_eq/image", 10, &ImageConverter::imageCallback, this);

        //kinect2_head/ir_rect_eq/image
        //kinect2_head/depth_rect/image
      image_pub = it_.advertise("/image_converter/output_video", 1);


      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }


void boxCallback(const opt_msgs::DetectionArray::ConstPtr& msg){



    int nbOfTracks=msg->detections.size();
     if (nbOfTracks>0) {
    xmin=msg->detections[0].box_2D.x;
    ymin=msg->detections[0].box_2D.y;
    xmax=xmin+msg->detections[0].box_2D.width;
    ymax=ymin+msg->detections[0].box_2D.height;
    personCentroid=msg->detections[0].centroid.z;
    xcentroid=msg->detections[0].centroid.x;
    ycentroid=msg->detections[0].centroid.y;


//    ROS_INFO("Depth: %d", depth);
    ROS_INFO("xmin: %f", xmin);
    ROS_INFO("ymin: %f", ymin);
    ROS_INFO("xmax: %f", xmax);
    ROS_INFO("ymax: %f", ymax);
    ROS_INFO("personCentroid: %f", personCentroid);
    ROS_INFO("xcentroid: %f", xcentroid);
    ROS_INFO("ycentroid: %f", ycentroid);


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

    double xc=(xmin+xmax)/2;
    double yc=ymin+(ymax-ymin)/3; //the 1/3 upper body

    double depth = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.
    ROS_INFO("depth: %f", depth);

  //  cv::circle(cv_ptr->image, cv::Point(xc, yc), 50, cv::Scalar(255,255,255));
    cv::rectangle(cv_ptr->image, cv::Point(xmin, ymin),	cv::Point(xmax, ymax), cv::Scalar(0,0,0), CV_FILLED, 8);
              cv::putText(cv_ptr->image, "test", cv::Point(xc, yc), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1.7, CV_AA);

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

////////////////////////////////////
/*

namespace enc = sensor_msgs::image_encodings;
//double xmin;
//double ymin;
//double xmax;
//double ymax;
//double personCentroid;
//double xcentroid;
//double ycentroid;

static const std::string OPENCV_WINDOW = "Image window";

class Listener
{
public:
  void boxCallback(const opt_msgs::DetectionArray::ConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

//cv::Mat roi( depth_image, cv::Rect( left, top, width, height ) );


void Listener::boxCallback(const opt_msgs::DetectionArray::ConstPtr& msg){
    int nbOfTracks=msg->detections.size();
     if (nbOfTracks>0) {
    double xmin=msg->detections[0].box_2D.x;
    double ymin=msg->detections[0].box_2D.y;
    double xmax=xmin+msg->detections[0].box_2D.width;
    double ymax=ymin+msg->detections[0].box_2D.height;
    double personCentroid=msg->detections[0].centroid.z;
    double xcentroid=msg->detections[0].centroid.x;
    double ycentroid=msg->detections[0].centroid.y;


//    ROS_INFO("Depth: %d", depth);
    ROS_INFO("xmin: %f", xmin);
    ROS_INFO("ymin: %f", ymin);
    ROS_INFO("xmax: %f", xmax);
    ROS_INFO("ymax: %f", ymax);
    ROS_INFO("personCentroid: %f", personCentroid);
    }
}


void Listener::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    double depth = cv_ptr->image.at<short int>(cv::Point(300,700));//you can change 240,320 to your interested pixel
    ROS_INFO("depth: %f", depth);


}

int main(int argc, char **argv){


    ros::init(argc, argv, "listener_class");
    ros::NodeHandle n;
    Listener Listener;

     ros::Subscriber sub = n.subscribe("/kinect2_head/depth_rect/image", 10, &Listener::imageCallback, &Listener);
     ros::Subscriber sub1 = n.subscribe("/detector/detections", 10, &Listener::boxCallback, &Listener);

     //kinect2_head/ir_rect_eq/image
//kinect2_head/depth_rect/image
     ros::spin();
      return 0;
}
*/











