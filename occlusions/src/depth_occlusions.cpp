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

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    ros::Subscriber person_sub = n.subscribe("/tracker/tracks", 10, &ImageConverter::boxCallback, this);  //get the track parameters
    ros::Publisher side= (n.advertise<occlusions::sideOcclusions>("occlusions/sideOcclusions",10));   //publish the 6 boolians

    double xmin=0;                 //top-left of the BBC (Bounding Box Coordinates)
    double ymin=0;                 //bottom-left of the BBC
    double xmax=0;                 //top-right of the BBC
    double ymax=0;                 //bottom-right of the BBC

    double personCentroid;         //from Open_PTrack trackers
    double distance;               //from Open_PTrack trackers- distance in meters to the detect person
    double confidence;             //from Open_PTrack trackers- from the SVM+HOG classifier- confidence for a real person
    double age;                    //from Open_PTrack trackers
    double height;                 //from Open_PTrack trackers
    double xc;                     //center of the BBC
    float depth;                   //pixel depth value at xc,yc
    float personDepth;             //distance*1000

 //   double depthTheshold=800.0;
    double depthTheshold=3.0;      //threshold for detect closer pixels from the personDepth value

    bool validTrack;               //good track
    int nbOfTracks;                //number of ID tracks
    float normalize;               //normalize the depth value


public:
    ImageConverter()
      : it_(n)
    {
      image_sub = it_.subscribe("/kinect2_head/depth_rect/image", 10, &ImageConverter::imageCallback, this);   //depth image (same as Open_PTrack uses)
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


void imageCallback(const sensor_msgs::ImageConstPtr& msg)     //working on the depth image
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


    xc=(xmin+xmax)/2;                        //center of the BBC
//    depth = cv_ptr->image.at<short int>(cv::Point(xc,yc));//milimeters for topic kinect2_head/depth_rect/image. and -XXXXX for topic kinect2_head/ir_rect_eq/image  -the amount of infrared light reflected back to the camera.
    personDepth=distance*1000;               //to avoid an error from calculate the depth only from one pixel, it's better to calculate from the all distance from the person and multipile by 1000 to get milimeters
    depth=personDepth*255/pow(2,16);         //to normalize to 255

 //   ROS_INFO("personDepth: %f", personDepth);

//left and right detections
    int downCut= round((ymax-ymin)/8);                          //to cut the lower part of the person for reduce floor alarm
    int smallOcclusions= round(((xc-xmin)/3)*(ymax-ymin)*7/8);  //detect small occlusion
    int bigOcclusions= round(((xc-xmin)/2)*(ymax-ymin)*7/8);    //detect big occlusion
    int marginAdd= round(10/distance);                          //add margin depend on distance

    // size of a pixel depend on depth
   // x = (X - 3.3931e+02) * z / 5.9421e+02
   // y = (Y - 2.4274e+02) * z / 5.9421e+02

    int countLeft= 0;
    int countRight= 0;

    bool smallLeftOcclusions= false;    //detect occlusion from the left (point of view of the robot)
    bool bigLeftOcclusions= false;      //detect occlusion from the left (point of view of the robot)
    bool LeftWall= false;               //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
    int countLeftWall= 0;

    bool smallRightOcclusions= false;   //detect occlusion from the left (point of view of the robot)
    bool bigRightOcclusions= false;     //detect occlusion from the left (point of view of the robot)
    bool RightWall= false;              //detect a "tall" occlusion from the left (point of view of the robot) like a wall for all the y axis of the person bounding box
    int countRightWall= 0;


   //left
        for (short int i=xmin-marginAdd;i<xc-5;i++){                       //over each colom from the left with margin up to the center minus 5
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
        for (short int i=xc+5;i<xmax+marginAdd;i++){                       //over each colom from the center plus 5 up to the right with margin
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
 //           ROS_INFO("LeftOcclusions: small %d", countLeft);
        }
        else  if (countLeft>bigOcclusions){                                //if number of pixels from the left are more than bigLeftOcclusions this is a bigLeft
            bigLeftOcclusions=true;
//            ROS_INFO("LeftOcclusions: big %d", countLeft);
        }
        else {//ROS_INFO("LeftOcclusions: false %d", countLeft);             //else there is no occlusions
  //      ROS_INFO("depth: %f", depth);
        }

 //       ROS_INFO("RightWall: %d", RightWall);
 //       ROS_INFO("LeftWall: %d", LeftWall);

        if (countRight>smallOcclusions&&countRight<bigOcclusions){         //if number of pixels from the right are between smallRightOcclusions and bigRightOcclusions this is a smallRight
            smallRightOcclusions=true;
 //           ROS_INFO("RightOcclusions: small %d", countRight);
        }
        else  if (countRight>bigOcclusions){                               //if number of pixels from the right are more than bigRightOcclusions this is a bigRight
            bigRightOcclusions=true;
 //           ROS_INFO("RightOcclusions: big %d", countRight);
        }
        else {//ROS_INFO("RightOcclusions: false %d", countRight);           //else there is no occlusions
    //    ROS_INFO("depth: %f", depth);
        }


  //publish the boolians variables
    bool_msg.bigLeft=bigLeftOcclusions;
    bool_msg.smallLeft=smallLeftOcclusions;
    bool_msg.wallLeft=LeftWall;
    bool_msg.bigRight=bigRightOcclusions;
    bool_msg.smallRight=smallRightOcclusions;
    bool_msg.wallRight=RightWall;

    side.publish(bool_msg);

    ROS_INFO("personDepth: %f", personDepth);
    ROS_INFO("distance: %f", distance);
    ROS_INFO("smallLeftOcclusions: %d", smallLeftOcclusions);
    ROS_INFO("bigLeftOcclusions: %d", bigLeftOcclusions);
    ROS_INFO("LeftWall: %d", LeftWall);
    ROS_INFO("countLeft: %f", countLeft);
    ROS_INFO("smallRightOcclusions: %d", smallRightOcclusions);
    ROS_INFO("bigRightOcclusions: %d", bigRightOcclusions);
    ROS_INFO("RightWall: %d", RightWall);
    ROS_INFO("countRight: %f", countRight);



    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::NodeHandle n;

     ros::spin();
      return 0;
}






