
#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "LaserPublisher.h"
#include "ArTimeToROSTime.h"


// TODO publish pointcloud of cumulative readings in separate topic?
// TODO generic pointcloud sensor publisher (seprate point cloud stuff there)
// TODO make  similar sonar publisher?

LaserPublisher::LaserPublisher(ArLaser *_l, ros::NodeHandle& _n, bool _broadcast_tf, const std::string& _tf_frame, const std::string& _parent_tf_frame, const std::string& _global_tf_frame) :
  laserReadingsCB(this, &LaserPublisher::readingsCB),
  node(_n),
  laser(_l),
  tfname(_tf_frame),
  parenttfname(_parent_tf_frame),
  globaltfname(_global_tf_frame),
  broadcast_tf(_broadcast_tf)
{
  assert(_l);
  laser->lockDevice();
  laser->addReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  std::string laserscan_name(laser->getName());
  laserscan_name += "_laserscan";
  std::string pointcloud_name(laser->getName());
  pointcloud_name += "_pointcloud";
  laserscan_pub = node.advertise<sensor_msgs::LaserScan>(laserscan_name, 20);
  pointcloud_pub = node.advertise<sensor_msgs::PointCloud>(pointcloud_name, 50);

  tf::Quaternion q;
  if(laser->hasSensorPosition())
  {
    lasertf.setOrigin(tf::Vector3(laser->getSensorPositionX()/1000.0, laser->getSensorPositionY()/1000.0, laser->getSensorPositionZ()/1000.0));
    q.setRPY(0, 0, ArMath::degToRad(laser->getSensorPositionTh()));
  }
  else
  {
    lasertf.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, 0);
  }
  lasertf.setRotation(q);
  

  laserscan.header.frame_id = "laser_frame";
  laserscan.angle_min = ArMath::degToRad(-120);
  laserscan.angle_max = ArMath::degToRad(120);
  //Modif for proper angle increment
  laserscan.angle_increment = ArMath::degToRad(0.50);
  //laserscan.time_increment = ?
  laserscan.range_min = 0; //laser->getMinRange() / 1000.0;
  laserscan.range_max = laser->getMaxRange() / 1000.0;
  pointcloud.header.frame_id = globaltfname;

  readingsCallbackTime = new ArTime;
}

LaserPublisher::~LaserPublisher()
{
  laser->lockDevice();
  laser->remReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  delete readingsCallbackTime;
}

void LaserPublisher::readingsCB()
{
  //printf("readingsCB(): %lu ms since last readingsCB() call.\n", readingsCallbackTime->mSecSince());
  assert(laser);
  laser->lockDevice();
  publishLaserScan();
  publishPointCloud();
  laser->unlockDevice();
  if(broadcast_tf)
    transform_broadcaster.sendTransform(tf::StampedTransform(lasertf, convertArTimeToROS(laser->getLastReadingTime()), parenttfname, tfname));
  //readingsCallbackTime->setToNow();
}

void LaserPublisher::publishLaserScan()
{
  laserscan.header.stamp = convertArTimeToROS(laser->getLastReadingTime());
  const std::list<ArSensorReading*> *readings = laser->getRawReadings(); 
  assert(readings);
  //printf("laserscan: %lu readings\n", readings->size());

  //Modified to ignore -135 to -120 and 120 to 135 deg readings and to inverse readings to take into account the inverted position of the sensor
  laserscan.ranges.resize(480);
  size_t n = 0;
  int i=479;
  for(std::list<ArSensorReading*>::const_iterator r = readings->begin(); r != readings->end(); ++r)
  {
    assert(*r);
    if (n>30 && n<=510){
    laserscan.ranges[i] = (*r)->getRange() / 1000.0;
    i--;
    }
    ++n;
  }
  laserscan_pub.publish(laserscan);
}

void LaserPublisher::publishPointCloud()
{
  assert(laser);
  pointcloud.header.stamp = convertArTimeToROS(laser->getLastReadingTime());
  assert(laser->getCurrentBuffer());
  const std::list<ArPoseWithTime*> *p = laser->getCurrentRangeBuffer()->getBuffer();
  assert(p);
  pointcloud.points.resize(p->size());
  size_t n = 0;
  for(std::list<ArPoseWithTime*>::const_iterator i = p->begin(); i != p->end(); ++i)
  {
    assert(*i);
    pointcloud.points[n].x = (*i)->getX() / 1000.0;
    pointcloud.points[n].y = (*i)->getY() / 1000.0;
    pointcloud.points[n].z = (laser->hasSensorPosition() ?  laser->getSensorPositionZ() / 1000.0 : 0.0);
    ++n;
  }
  pointcloud_pub.publish(pointcloud);
}
  
  
