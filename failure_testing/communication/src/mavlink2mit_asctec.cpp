/*
 * FILE: mavlink2mit_asctec.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2MIT_AscTec.h"

// defines
#define ROS_PKG "flightTest"
#define ROS_NODE "mmaTranslation"

// constructor
mmaTranslation::mmaTranslation():nh(){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();

  // initialize timers
  initTimers();

  // start main thread
  startThread();
}

// destructor
mmaTranslation::~mmaTranslation(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start parse serial instance
  mmaTranslation mmaFlight;

  // keep spinning
  ros::spin();

  // log stop
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
