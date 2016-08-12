/*
 * FILE: acknowledge_control_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/acknowledge_control.h"

// initialize parameters
void Acknowledge::initParams(){ 
  bytes = seq = full = count = stuck_count = 0;
  at_enable = at_request = rf_test = ack_data = false;
  first = true;

  // set robot or controlling station
  pnh.param("robot",robot,false);

  rdata.layout.dim.push_back(std_msgs::MultiArrayDimension());
}

// initialize subscribers
void Acknowledge::initSubscribers(){
  // subscribe to data from odroid
  data_sub = nh.subscribe<std_msgs::UInt8MultiArray>("rf_rx_data",10,&Acknowledge::callbackData,this);

  // subscribe to test phase topic
  test_sub = nh.subscribe<gps_testing::Move>("resume_test",5,&Acknowledge::callbackRSSI,this);
}

// initialize publishers
void Acknowledge::initPublishers(){
  // publish to acknowledgement
  ack_pub = nh.advertise<gps_testing::Ack>("ack",1,true);

  // publish data to odroid
  data_pub = nh.advertise<std_msgs::UInt8MultiArray>("rf_tx_data",1,true);

  // publish to test phase topic
  test_pub = nh.advertise<gps_testing::Move>("resume_test",1,true); 
}

// initialize timers
void Acknowledge::initTimers(){
  rf_data = nh.createTimer(ros::Duration(3.0),&Acknowledge::rfData,this);
}

