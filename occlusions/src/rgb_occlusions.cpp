
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
occlusions::sideOcclusions bool_msg;            //6 boolians variables (big,small,wall- for right and left)

double AgeThreshold=0;                          //how "old" is the ID
double ConfidenceTheshold=0.6; //1.1            //from the SVM+HOG classifier- confidence for a real person
double HeightTheshold=1.4;                      //height in meter of the person

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

    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);  //get the track parameters
    ros::Publisher side= (n.advertise<occlusions::sideOcclusions>("occlusions/sideOcclusions",10));       //publish the 6 boolians

    double xmin=0;                      //top-left of the BBC (Bounding Box Coordinates)from the DEPTH image
    double ymin=0;                      //bottom-left of the BBC from the DEPTH image
    double xmax=0;                      //top-right of the BBC from the DEPTH image
    double ymax=0;                      //bottom-right of the BBC from the DEPTH image
    double xcenter;                     //the center of the box in x axis at the DEPTH image

    double distance;                    //from Open_PTrack trackers- distance in meters to the detect person
    double confidence;                  //from Open_PTrack trackers- from the SVM+HOG classifier- confidence for a real person
    double age;                         //from Open_PTrack trackers
    double height;                      //from Open_PTrack trackers

    double rgbxmin=0;                   //top-left of the BBC from the MONO image
    double rgbymin=0;                   //bottom-left of the BBC from the MONO image
    double rgbxmax=0;                   //top-right of the BBC from the MONO image
    double rgbymax=0;                   //bottom-right of the BBC from the MONO image
    double xc;                          //the center of the box in x axis from the MONO image
    double yc;                          //the 1/3 upper body from the MONO umage
    double xcBox;                       //the center of the box in the ROI

    bool validTrack;                    //good track
    int nbOfTracks;                     //number of ID tracks
    bool LeftWall= false;               //detect left wall
    bool RightWall= false;              //detect right wall

public:
    ImageConverter()
      : it_(n)
    {
      image_sub = it_.subscribe("/kinect2_head/mono_rect/image", 10, &ImageConverter::imageCallback, this);  //get the mono_rect image (gray image)
      image_pub = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }


void boxCallback(const opt_msgs::TrackArray::ConstPtr& msg){              //get all the tracks parameters from Open_PTrack

    bool validTrack=false;

    int nbOfTracks=msg->tracks.size();
     if (nbOfTracks>0) {
         for(int i=0;i<nbOfTracks && !validTrack;i++){
             //oldest track which is older than the age threshold and above the confidence threshold and above the height threshold
             if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold)){
    xmin=msg->tracks[i].box_2D.x;                //top-left of the BBC (Bounding Box Coordinates)from the DEPTH image
    ymin=msg->tracks[i].box_2D.y;                //bottom-left of the BBC from the DEPTH image
    xmax=xmin+msg->tracks[i].box_2D.width;       //top-right of the BBC from the DEPTH image
    ymax=ymin+msg->tracks[i].box_2D.height;      //bottom-right of the BBC from the DEPTH image
    distance=msg->tracks[i].distance;            //from Open_PTrack trackers- distance in meters to the detect person
    confidence=msg->tracks[i].confidence;        //from Open_PTrack trackers- from the SVM+HOG classifier- confidence for a real person
    height=msg->tracks[i].height;                //from Open_PTrack trackers
    age=msg->tracks[i].age;                      //from Open_PTrack trackers

validTrack=true;

            }
        }
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)             //working on the depth image
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);                            //now cv_ptr is the matrix of the image

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat deleteMargin= cv::Mat::zeros(1080,1710,0);
    deleteMargin = cv_ptr->image(Rect(104,0,1710,1080)).clone();     //because there is a different FOV between depth and RGB i delete the 105 pixels from each side to reduce the FOV different

    cv::Size size(960,540);                                          //size of the depth image
    cv::resize(deleteMargin,deleteMargin,size);                      //resize the gray mono image to the depth image size because the resolotion of mono_rect is twice the resolotion of depth_rect

       xcenter=(xmin+xmax)/2;                                        //the center of the box in x axis at the DEPTH image
       rgbxmin=xmin*2+round(((270-xcenter)/3)-distance*2);           //top-left of the BBC from the MONO image with react to the center of the image because different FOV and with react to the distance
       rgbxmax=rgbxmin+(xmax-xmin)*1.4;                              //top-right of the BBC from the MONO image with more width to cover the all person
       rgbymin=ymin;                                                 //bottom-left of the BBC from the MONO image
       rgbymax=rgbymin+(ymax-ymin)*1.3;                              //bottom-right of the BBC from the MONO image with more for the legs but not the ground


    xc=(rgbxmin+rgbxmax)/2;                                          //the center of the box in x axis at the MONO image
    yc=ymin+(rgbymax-rgbymin)/3;                                     //the 1/3 upper body from the MONO umage
    ROS_INFO("xcenter: %f", xcenter);


    int marginAdd= round(50/distance);                              //add margin depend on distance
    LeftWall= false;                                                //detect a "tall" vertical occlusion from the left (point of view of the robot)- a wall
    RightWall= false;                                               //detect a "tall" vertical occlusion from the left (point of view of the robot)- a wall
 //   int smallOcclusions= round((xcBox-xmin*2+round(((270-xcenter)/2)-(distance*3)))/3);  //detect small occlusion
 //   int bigOcclusions= round((xcBox-xmin*2+round(((270-xcenter)/2)-(distance*3)))/2);    //detect big occlusion

    xcBox=((xmax-xmin)*1.4+marginAdd*2)/2;                          //the center of the box in the ROI
    cv::Mat temp= cv::Mat::zeros(540,960,0);                        //temp mat with the maximum size of the MONO image

    int top = (int) (0.01*temp.rows);                               //for add borders to the image
    int bottom = (int) (0.01*temp.rows);
    int left = (int) (0.01*temp.cols);
    int right = (int) (0.01*temp.cols);

 if(rgbxmin-marginAdd*2 >= 0 && rgbymin >= 0 && rgbxmax+marginAdd < deleteMargin.cols && rgbymax < deleteMargin.rows && (xmax-xmin)*1.4+marginAdd>0 && (ymax-ymin)*1.3>0){
    temp = deleteMargin(Rect(rgbxmin-marginAdd,rgbymin,(xmax-xmin)*1.4+marginAdd,(ymax-ymin)*1.3)).clone();  //take only the BBC with the person with margin depend on distance
    }

    cv::GaussianBlur(temp, temp, cv::Size(3,3), 0);                                                     //gaussian blur 3*3
    cv::Canny(temp, temp, 50.0, 300.0, 3, false);                                                       //canny edge detector
    cv::Mat element1= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(-1,-1));       // 5*5 open element
    cv::Mat element2= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(-1,-1));       // 5*5 close element
    cv::dilate(temp, temp, element1);                                                                   //open the pixels
    cv::erode(temp, temp, element2);                                                                    //close the pixels

    cv::Mat temp2=temp;
    cv::cvtColor(temp2, temp2, cv::COLOR_GRAY2BGR);                                                     //color mat for show
    copyMakeBorder( temp2, temp2, top, bottom, left, right, BORDER_CONSTANT, cv::Scalar(255,255,255) ); //add white borders to help detect contours


        std::vector<std::vector<cv::Point> > contours;                                                  //vector of vector of points for contours
        cv::findContours(temp,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);                            //find contours
            for (int i=0; i<contours.size(); i++){
                if(contours[i].size()>(rgbymax-rgbymin)*1.5){                                           //only if the contour is bigger than the whole height of the ROI multipile 1.5
                cv::drawContours(temp2,contours,i,cv::Scalar(0,255,0),8,8);                             //draw green contours
                }
                else {contours.pop_back();}                                                             //delete small contours from the vector "contours"
            }

        cv::vector<Vec4i> lines;                                                                       //vector "lines" contain 4 argument for xStart, yStart, xEnd, yEnd for a line
          HoughLinesP(temp, lines, 1, CV_PI / 180, 50, (rgbymax-rgbymin)*0.5, 0 );                     //find straight lines with Hough that are bigger than the half height of the ROI

          for (size_t i = 0; i < lines.size(); i++)
          {
              cv::Vec4i l = lines[i];
              if (abs(l[0]-l[2])<(rgbxmax-rgbxmin)/10) {                                               //for only vertical lines depend on the width of the person box divide by 10 (only 1/10 size of the width)
                  line(temp2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 10, 4);         //draw red line
                  //left
                  if (((l[0]>0) && (l[0]<xcBox-5)) || ((l[2]>0) && (l[2]<xcBox-5))){                   //if the edges of the straight line is inside the ROI from the left to the center minus 5 it's a left wall
                      LeftWall=true;
                  }
                  //right
                  if (((l[0]>xcBox+5) && (l[0]<temp.cols)) || ((l[2]>xcBox+5) && (l[2]<temp.cols))){   //if the edges of the straight line is inside the ROI from the center plus 5 to the right it's a right wall
                      RightWall=true;
                  }
              }
              else {lines.pop_back();}                                                                 //delete all the other lines from the vector "lines"
          }

      // Draw a rectangle around the detected person:
      //      rectangle(temp,cv::Point(rgbxmin,rgbymin),cv::Point(rgbxmax,rgbymax),cv::Scalar(0,255,0), 10);
            rectangle(temp2,cv::Point(rgbxmin,rgbymin),cv::Point(rgbxmax,rgbymax),cv::Scalar(0,255,0), 10);

    // Update GUI Window
          cv::imshow(OPENCV_WINDOW, temp2);
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





////////////////////////////////////////work for the ALL image (0.5hz)
/*
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
              HoughLinesP(cv_ptr->image, lines, 1, CV_PI / 180, 50, (rgbymax-rgbymin)/3, 10 );  //find straight lines with Hough

              for (size_t i = 0; i < lines.size(); i++)
              {
                  cv::Vec4i l = lines[i];
         //         int ratioVertical= abs((l[0]-l[2])/(l[1]-l[3]));
         //         double ratioVertical= abs(l[2]/l[3]);
                  if (abs(l[0]-l[2])<(rgbxmax-rgbxmin)/10) {    //for only vertical lines depend on the width of the person box divide by 10 (only 1/10 size of the width)
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
*/
