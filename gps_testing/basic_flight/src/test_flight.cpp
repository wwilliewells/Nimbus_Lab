/*
 * FILE: test_flight.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016 ...
 */

// class include
#include "../include/test_flight.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "testFlight"

// constructor
TestFlight::TestFlight():nh(),pnh("~"){
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
TestFlight::~TestFlight(){ stopThread(0); }

// return true if landing is a success
bool TestFlight::getLast(){ return last; }

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start flight test instance
  TestFlight testflyte;
    
  // keep spinning until landing acheived
  while(!testflyte.getLast()){ ros::spinOnce(); }
    
  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);
  ros::shutdown();

  return 0;
}
