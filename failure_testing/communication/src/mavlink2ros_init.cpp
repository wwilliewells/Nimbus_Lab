/*
 * FILE: mavlink2ros_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2ros.h"

// initialize parameters
void mrTelemetry::initParams(){
  // initialize state
  state = START;
  switch_state = false;
  first = true;
 
  // initialize layout of transmission data
  message_tx.layout.dim.push_back(std_msgs::MultiArrayDimension());

  // initialize private real numbers
  seq = bytes = 2;
  c = size = 0;
}

// initialize subscribers
void mrTelemetry::initSubscribers(){
  // subscribe to communication robot_rx_data 
  robot_receive = nh.subscribe("robot_rx_data",200,&mrTelemetry::callbackRobotRx,this);
    
  // subscribe to outgoing topic mavlink_tx
  mavlink_transmit = nh.subscribe<flight_testing::Mavlink>("mavlink_tx",100,&mrTelemetry::callbackMavlinkTx,this);
}

// initialize publishers
void mrTelemetry::initPublishers(){
  // publish outgoing message to robot
  robot_transmit = nh.advertise<std_msgs::UInt8MultiArray>("robot_tx_data",1,true);

  // publish mavlink packet
  mavlink_receive = nh.advertise<flight_testing::Mavlink>("mavlink_rx",1,true);
}

