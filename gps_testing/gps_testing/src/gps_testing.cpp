/*
 * FILE: gps_testing.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016 ...
 */

// class include
#include "../include/gps_testing.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "gpsTest"

// constructor
GpsTest::GpsTest():nh(),pnh("~"){
  // initialize parameters
  initParams();ROS_INFO("parameteers initialized");

  // initialize publishers
  initPublishers();ROS_INFO("publishers initialized");

  // initialize subscribers
  initSubscribers();ROS_INFO("subscribers initialized");

  // initialize services

  // start main thread
  startThread(0);
}

// destructor
GpsTest::~GpsTest(){ stopThread(0); }

// return true if landing is a success
bool GpsTest::getLast(){ return last; }

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start flight test instance
  GpsTest globetest;
    
  // keep spinning until landing acheived
  while(!globetest.getLast()){ ros::spinOnce(); }
    
  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);
  ros::shutdown();

  return 0;
}
