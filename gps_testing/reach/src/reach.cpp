/*
 * FILE: reach.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/reach.h"

// defines
#define ROS_NODE "reach"

// constructor
reach::reach():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();
}

// destructor
reach::~reach(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start node instance
  reach stretch;

  // keep spinning until landing acheived
  ros::spin();

  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}

