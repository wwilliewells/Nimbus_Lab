/*
 * FILE: send_serial.cpp
 * AUTHOR: William Willie Wells
 * DATE: July 2016 ...
 */

// class include
#include "../include/send_serial.h"

// defines
#define ROS_PKG "gpsTest"
#define ROS_NODE "sendSerial"

// constructor
sendSerial::sendSerial():nh(){
  // initialize publishers
  initPublishers();

  // initialize subscribers
  initTimers();
}

// destructor
sendSerial::~sendSerial(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start instance
  sendSerial laser;
    
  // keep spinning until landing acheived
  ros::spin();
    
  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
