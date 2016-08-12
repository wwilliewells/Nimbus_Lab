/*
 * FILE: xbee_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/xbee.h"

// initialize parameters
void XBee::initParams(){
  total_bytes = bytes_received = len = seq = d_seq = 0;
  // set destination address
  pnh.param("receiver",rcvr,0x23);
  ROS_INFO("receiver id: %d",rcvr);
}

// initialize subscribers
void XBee::initSubscribers(){
  // subscribe to data from odroid
  data_sub = nh.subscribe<std_msgs::UInt8MultiArray>("rf_rx_data",1000,&XBee::callbackData,this);

  // subscribe to data from device(s)
  device_sub = nh.subscribe<std_msgs::UInt8MultiArray>("device_tx_data",10,&XBee::callbackDevice,this);
}

// initialize publishers
void XBee::initPublishers(){
  // publish data to odroid
  data_pub = nh.advertise<std_msgs::UInt8MultiArray>("rf_tx_data",1000,true);

  // publish to device receive -- contains position estimates
  device_pub = nh.advertise<std_msgs::UInt8MultiArray>("device_rx_data",1,true);

  // publish RSSI
  rssi_pub = nh.advertise<gps_testing::Ack>("ack",1,true);

  // publish byte log
  byte_pub = nh.advertise<gps_testing::Bytes>("byte",1,true);
}

// initialize timers
void XBee::initTimers(){
  dataStream = nh.createTimer(ros::Duration(0.25),&XBee::stream,this);
}

