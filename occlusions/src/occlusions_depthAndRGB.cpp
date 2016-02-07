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
occlusions::sideOcclusions bool_msg;  //6 boolians variables (big,small,wall- for right and left)

double AgeThreshold=0;  //how "old" is the ID
double ConfidenceTheshold=0.6; //1.1    //from the SVM+HOG classifier- confidence for a real person
double HeightTheshold=1.4;   //height in meter of the person (minimum)
double HeightMaxTheshold=2.0;   //height in meter of the person (maximum)


namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Publisher image_pub;

    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);  //get the track parameters
    ros::Publisher side= (n.advertise<occlusions::sideOcclusions>("occlusions/sideOcclusions",10));   //publish the 6 boolians

    double changeDepthToRGB=5.0;        //which distance to change between depth to RGB
    double xmin=0;                      //top-left of the BBC (Bounding Box Coordinates)from the DEPTH image
    double ymin=0;                      //bottom-left of the BBC from the DEPTH image
    double xmax=0;                      //top-right of the BBC from the DEPTH image
    double ymax=0;                      //bottom-right of the BBC from the DEPTH image
    double xcenter;                     //the center of the box in x axis at the DEPTH image

    double personCentroid;         //from Open_PTrack trackers
    double distance;                    //from Open_PTrack trackers- distance in meters to the detect person
    double confidence;                  //from Open_PTrack trackers- from the SVM+HOG classifier- confidence for a real person
    double age;                         //from Open_PTrack trackers
    double height;                      //from Open_PTrack trackers
    double xc;                     //center of the BBC
    float depth;                   //pixel depth value at xc,yc
    float personDepth;             //distance*1000

 //   double depthTheshold=800.0;
    double depthTheshold=3.0;      //threshold for detect closer pixels from the personDepth value

    double rgbxmin=0;                   //top-left of the BBC from the MONO image
    double rgbymin=0;                   //bottom-left of the BBC from the MONO image
    double rgbxmax=0;                   //top-right of the BBC from the MONO image
    double rgbymax=0;                   //bottom-right of the BBC from the MONO image
    double xcBox;                       //the center of the box in the ROI

    bool validTrack;               //good track
    int nbOfTracks;                //number of ID tracks
    float normalize;               //normalize the depth value
/*
    bool smallLeftOcclusions= false;    //detect occlusion from the left (point of view of the robot)
    bool bigLeftOcclusions= false;      //detect occlusion from the left (point of view of the robot)
    bool LeftWall= false;               //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
    bool smallRightOcclusions= false;   //detect occlusion from the left (point of view of the robot)
    bool bigRightOcclusions= false;     //detect occlusion from the left (point of view of the robot)
    bool RightWall= false;              //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
*/

public:
    ImageConverter()
      : it_(n)
    {
      image_sub1 = it_.subscribe("/kinect2_head/depth_rect/image", 10, &ImageConverter::imageDepthCallback, this);   //depth image (same as Open_PTrack uses)
      image_sub2 = it_.subscribe("/kinect2_head/mono_rect/image", 10, &ImageConverter::imageRGBCallback, this);  //get the mono_rect image (gray image)

      image_pub = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }


void boxCallback(const opt_msgs::TrackArray::ConstPtr& msg){    //get all the tracks parameters from Open_PTrack

    bool validTrack=false;

    int nbOfTracks=msg->tracks.size();
     if (nbOfTracks>0) {
         for(int i=0;i<nbOfTracks && !validTrack;i++){
             //oldest track which is older than the age threshold and above the confidence threshold and above the height threshold and under max height threshold
             if ((msg->tracks[i].age>AgeThreshold) && (msg->tracks[i].confidence>ConfidenceTheshold) && (msg->tracks[i].height>HeightTheshold) && (msg->tracks[i].height<HeightMaxTheshold)){
    xmin=msg->tracks[i].box_2D.x;             //top-left of the BBC (Bounding Box Coordinates)
    ymin=msg->tracks[i].box_2D.y;             //bottom-left of the BBC
    xmax=xmin+msg->tracks[i].box_2D.width;    //top-right of the BBC
    ymax=ymin+msg->tracks[i].box_2D.height;   //bottom-right of the BBC
    distance=msg->tracks[i].distance;         //from Open_PTrack trackers- distance in meters to the detect person
    confidence=msg->tracks[i].confidence;     //from Open_PTrack trackers- from the SVM+HOG classifier- confidence for a real person
    height=msg->tracks[i].height;             //from Open_PTrack trackers
    age=msg->tracks[i].age;                   //from Open_PTrack trackers

validTrack=true;

            }
        }
    }
}


void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)     //working on the depth image
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);   //now cv_ptr is the matrix
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    //      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //      cv::waitKey(3);

          image_pub.publish(cv_ptr->toImageMsg());      // Output modified video stream

          bool smallLeftOcclusions= false;    //detect occlusion from the left (point of view of the robot)
          bool bigLeftOcclusions= false;      //detect occlusion from the left (point of view of the robot)
          bool LeftWall= false;               //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
          bool smallRightOcclusions= false;   //detect occlusion from the left (point of view of the robot)
          bool bigRightOcclusions= false;     //detect occlusion from the left (point of view of the robot)
          bool RightWall= false;              //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box

if(distance<changeDepthToRGB){
    xc=(xmin+xmax)/2;                        //center of the BBC
//    depth = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.
    personDepth=distance*1000;               //to avoid an error from calculate the depth only from one pixel, it's better to calculate from the all distance from the person and multipile by 1000 to get milimeters
    depth=personDepth*255/pow(2,16);         //to normalize to 255

    ROS_INFO("personDepth: %f", personDepth);

//left and right detections
    int downCut= round((ymax-ymin)/8);                          //to cut the lower part of the person for reduce floor alarm
    int smallOcclusions= round(((xc-xmin)/3)*(ymax-ymin)*7/8);  //detect small occlusion
    int bigOcclusions= round(((xc-xmin)/2)*(ymax-ymin)*7/8);    //detect big occlusion
    int marginAddDepth= round(10/distance);                          //add margin depend on distance

    // size of a pixel depend on depth
   // x = (X - 3.3931e+02) * z / 5.9421e+02
   // y = (Y - 2.4274e+02) * z / 5.9421e+02

    int countLeft= 0;
    int countRight= 0;
    int countLeftWall= 0;
    int countRightWall= 0;

   //left
        for (short int i=xmin-marginAddDepth;i<xc-5;i++){                       //over each colom from the left with margin up to the center minus 5
            countLeftWall= 0;
            for (short int j=ymin;j<ymax-downCut;j++){                     //over all the specific colom from up to down without the lower part to avoid the floor
        normalize=cv_ptr->image.at<short int>(cv::Point(i,j));             //get the depth value for each pixel
        normalize=normalize*255/pow(2,16);                                 //normalize the depth value to 0-255

                if (normalize<(depth-depthTheshold)){                      //closer than the personDepth
                   countLeft++;
                   countLeftWall++;
                   if (countLeftWall==ymax-downCut-ymin){LeftWall=true;}   //if the all colom is closer than this is a wall
               }
        }
    }

   //right
        for (short int i=xc+5;i<xmax+marginAddDepth;i++){                       //over each colom from the center plus 5 up to the right with margin
            countRightWall= 0;
            for (short int j=ymin;j<ymax-downCut;j++){                     //over all the specific colom from up to down without the lower part to avoid the floor
        normalize=cv_ptr->image.at<short int>(cv::Point(i,j));             //get the depth value for each pixel
        normalize=normalize*255/pow(2,16);                                 //normalize the depth value to 0-255

                if (normalize<(depth-depthTheshold)){                      //closer than the personDepth
                   countRight++;
                   countRightWall++;
                   if (countRightWall==ymax-downCut-ymin){RightWall=true;} //if the all colom is closer than this is a wall
               }
        }
    }

        if (countLeft>smallOcclusions&&countLeft<bigOcclusions){           //if number of pixels from the left are between smallLeftOcclusions and bigLeftOcclusions this is a smallLeft
            smallLeftOcclusions=true;
            ROS_INFO("LeftOcclusions: small %d", countLeft);
        }
        else  if (countLeft>bigOcclusions){                                //if number of pixels from the left are more than bigLeftOcclusions this is a bigLeft
            bigLeftOcclusions=true;
            ROS_INFO("LeftOcclusions: big %d", countLeft);
        }
        else {ROS_INFO("LeftOcclusions: false %d", countLeft);             //else there is no occlusions
  //      ROS_INFO("depth: %f", depth);
        }

        ROS_INFO("RightWall: %d", RightWall);
        ROS_INFO("LeftWall: %d", LeftWall);

        if (countRight>smallOcclusions&&countRight<bigOcclusions){         //if number of pixels from the right are between smallRightOcclusions and bigRightOcclusions this is a smallRight
            smallRightOcclusions=true;
            ROS_INFO("RightOcclusions: small %d", countRight);
        }
        else  if (countRight>bigOcclusions){                               //if number of pixels from the right are more than bigRightOcclusions this is a bigRight
            bigRightOcclusions=true;
            ROS_INFO("RightOcclusions: big %d", countRight);
        }
        else {ROS_INFO("RightOcclusions: false %d", countRight);           //else there is no occlusions
    //    ROS_INFO("depth: %f", depth);
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

void imageRGBCallback(const sensor_msgs::ImageConstPtr& msg)     //working on the depth image
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

    // Update GUI Window
    //      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //      cv::waitKey(3);

          image_pub.publish(cv_ptr->toImageMsg());      // Output modified video stream

          bool smallLeftOcclusions= false;    //detect occlusion from the left (point of view of the robot)
          bool bigLeftOcclusions= false;      //detect occlusion from the left (point of view of the robot)
          bool LeftWall= false;               //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
          bool smallRightOcclusions= false;   //detect occlusion from the left (point of view of the robot)
          bool bigRightOcclusions= false;     //detect occlusion from the left (point of view of the robot)
          bool RightWall= false;              //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box

if(distance>changeDepthToRGB){
    cv::Mat deleteMargin= cv::Mat::zeros(1080,1710,0);
    deleteMargin = cv_ptr->image(Rect(104,0,1710,1080)).clone();     //because there is a different FOV between depth and RGB i delete the 105 pixels from each side to reduce the FOV different

    cv::Size size(960,540);                                          //size of the depth image
    cv::resize(deleteMargin,deleteMargin,size);                      //resize the gray mono image to the depth image size because the resolotion of mono_rect is twice the resolotion of depth_rect

       xcenter=(xmin+xmax)/2;                                        //the center of the box in x axis at the DEPTH image
       rgbxmin=xmin*2+round(((270-xcenter)/3)-distance*2);           //top-left of the BBC from the MONO image with react to the center of the image because different FOV and with react to the distance
       rgbxmax=rgbxmin+(xmax-xmin)*1.4;                              //top-right of the BBC from the MONO image with more width to cover the all person
       rgbymin=ymin;                                                 //bottom-left of the BBC from the MONO image
       rgbymax=rgbymin+(ymax-ymin)*1.3;                              //bottom-right of the BBC from the MONO image with more for the legs but not the ground

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







