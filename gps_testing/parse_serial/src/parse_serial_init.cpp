/*
 * FILE: parse_serial_init.cpp
 * AUTHOR: William Willie Wells modified from code written 
 *         by Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2016
 */

// class include
#include "../include/parse_serial.h"

// initialize parameters
void ParseSerial::initParams(){
  // initialize private booleans
  //flying = false;

  // initialize state
  state = NOT_SYN;

  // initialize private real numbers
  size = 1;
  c = 0;
  value = 0;
}

// initialize subscribers
void ParseSerial::initSubscribers(){
  // subscribe to communication serial port receiver
  serial_rx = nh.subscribe("device_rx_data",10,&ParseSerial::callbackSerialRx,this);

  // subscribe to  state 
  //flight = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",10,&ParseSerial::callbackState,this);

  // subscribe to task
  //task = nh.subscribe<collab_msgs::SubjectPose>("task_waypose",10,&ParseSerial::callbackTask,this);
}

// initialize publishers
void ParseSerial::initPublishers(){
  // publish each sensor value on separate topic
  sensor = nh.advertise<gps_testing::Sensor>("sensor_data",10,true);
}

