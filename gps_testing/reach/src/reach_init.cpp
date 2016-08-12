/*
 * FILE: reach_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/reach.h"

// initialize parameters
void reach::initParams(){
  count = decode_nmea = decode_llh = 0;
}

// initialize subscribers
void reach::initSubscribers(){
  // subscribe to
  reach_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/r/reach_tx_data",1000,&reach::callbackReach,this);
}

// initialize publishers
void reach::initPublishers(){
  // publish to 
  reach_pub = nh.advertise<std_msgs::UInt8MultiArray>("/r/reach_gps",1000,true);
}

