/*
 * FILE: parse_type_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/parse_type.h"

// initialize parameters
void parse_type::initParams(){}

// initialize subscribers
void parse_type::initSubscribers(){
  // subscribe to
  rx_sub = nh.subscribe<std_msgs::UInt8MultiArray>("rx_data",100,&parse_type::callbackType,this);
}

// initialize publishers
void parse_type::initPublishers(){
  // publish to rssi
  rssi_pub = nh.advertise<gps_testing::Rssi>("rssi",100,false);

  // publish to device
  device_pub = nh.advertise<std_msgs::UInt8MultiArray>("device_rx_data",100,false);
}

