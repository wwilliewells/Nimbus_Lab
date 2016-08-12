/*
 * File: reach.h
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

#ifndef _REACH_H_
#define _REACH_H_

// ros includes
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

// package message include
#include <gps_testing/Gps.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
//#define PI

// class
class reach{
  public:
    // con(de)structors
    reach();
    ~reach();
  private:
    void callbackReach(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // ros objects
    ros::NodeHandle nh, pnh;
    // ros publisher
    ros::Publisher reach_pub;
    // ros subscribers
    ros::Subscriber reach_sub;

    // class messages
    gps_testing::Gps gps_rtk;

    // in(ex)ternal parameters
    uint8_t count;
    uint8_t decode_nmea;
    uint8_t decode_llh;
};
#endif
