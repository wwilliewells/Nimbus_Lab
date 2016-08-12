/*
 * FILE: basic_flight.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016 ...
 */

// class include
#include "../include/basic_flight.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "basicFlight"

// constructor
BasicFlight::BasicFlight():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();

  // initialize services

  // start main thread
  //startThread(0);
}

// destructor
BasicFlight::~BasicFlight(){ /*stopThread(0);*/ }

// return true if landing is a success
//bool BasicFlight::getLast(){ return last; }

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start flight test instance
  BasicFlight flyte;
    
  // keep spinning until landing acheived
  //while(!flyte.getLast()){ ros::spinOnce(); }
  ros::spin();
    
  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);
  //ros::shutdown();

  return 0;
}
