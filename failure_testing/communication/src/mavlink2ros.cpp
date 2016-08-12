/*
 * FILE: mavlink2ros.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2ros.h"

// defines
#define ROS_PKG "flightTest"
#define ROS_NODE "mrTelemetry"

// constructor
mrTelemetry::mrTelemetry():nh(){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();
}

// destructor
mrTelemetry::~mrTelemetry(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start parse serial instance
  mrTelemetry mavtoros;

  // keep spinning
  ros::spin();

  // log stop
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
