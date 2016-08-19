/*
 * FILE: parse_type.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/parse_type.h"

// defines
#define ROS_NODE "parse_type"

// constructor
parse_type::parse_type():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();
}

// destructor
parse_type::~parse_type(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start node instance
  parse_type name;

  // keep spinning until landing acheived
  ros::spin();

  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
