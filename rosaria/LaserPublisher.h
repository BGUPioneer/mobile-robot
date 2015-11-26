#ifndef ROSARIALASERPUBLISHER_H
#define ROSARIALASERPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

class ArLaser;
class ArTime;

//Modified "/odom" to "/base_frame"
class LaserPublisher
{
public:
  LaserPublisher(ArLaser* _l, ros::NodeHandle& _n, bool _broadcast_transform = true, const std::string& _tf_frame = "laser", const std::string& _parent_tf_frame = "base_link", const std::string& _global_tf_frame = "base_link");
  ~LaserPublisher();
protected:
  void readingsCB();
  void publishLaserScan();
  void publishPointCloud();

  ArFunctorC<LaserPublisher> laserReadingsCB;
  ros::NodeHandle& node;
  ArLaser *laser;
  ros::Publisher laserscan_pub, pointcloud_pub;
  sensor_msgs::LaserScan laserscan;
  sensor_msgs::PointCloud  pointcloud;
  std::string tfname;
  std::string parenttfname;
  std::string globaltfname;
  tf::Transform lasertf;
  tf::TransformBroadcaster transform_broadcaster;
  bool broadcast_tf;

  ArTime *readingsCallbackTime;
};

#endif
