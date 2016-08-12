/*
 * FILE: gps_device.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016 ...
 */

// class include
#include "../include/gps_device.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "gpsDevice"

// constructor
gpsDevice::gpsDevice():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();
}

// destructor
gpsDevice::~gpsDevice(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start flight test instance
  gpsDevice rtk_device;
    
  // keep spinning until landing acheived
  ros::spin();
    
  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
