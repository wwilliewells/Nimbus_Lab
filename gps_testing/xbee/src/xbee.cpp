/*
 * FILE: xbee.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016 ...
 */

// class include
#include "../include/xbee.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "xbee"

// constructor
XBee::XBee():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();

  // initialize timers
  initTimers();
}

// destructor
XBee::~XBee(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start flight test instance
  XBee zigbee;

  // keep spinning until landing acheived
  ros::spin();

  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}

