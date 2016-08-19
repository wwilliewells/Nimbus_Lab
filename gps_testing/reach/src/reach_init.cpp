/*
 * FILE: reach_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/reach.h"

// initialize parameters
void reach::initParams(){ count = decode_nmea = decode_llh = 0; }

// initialize subscribers
void reach::initSubscribers(){
  // subscribe to pose estimate
  reach_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/r/reach_tx_data",20,
    &reach::callbackReach,this);
}

// initialize publishers
void reach::initPublishers(){
  // publish to decoded pose topic
  reach_pub = nh.advertise<gps_testing::Gps>("/r/reach_gps",10,true);
}

