/*
 * FILE: acknowledge_control.h
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

#ifndef _ACKNOWLEDGE_CONTROL_H_
#define _ACKNOWLEDGE_CONTROL_H_

// ros includes
#include "ros/ros.h"
#include <std_msgs/UInt8MultiArray.h>

// package message include
#include <gps_testing/Ack.h>
#include <gps_testing/Move.h>

// c++ includes
#include <boost/thread/thread.hpp>
#include <cmath>

// namespace
using namespace std;

// defines

// class
class Acknowledge{
  public:
    // con(de)structors
    Acknowledge();
    ~Acknowledge();

  private:
    // callbacks
    void callbackRSSI(const boost::shared_ptr<gps_testing::Move const>&);
    void callbackData(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);

    // timers
    void rfData(const ros::TimerEvent&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    void initTimers();

    // ros objects
    ros::NodeHandle nh, pnh;
    ros::Timer rf_data;

    // ros publisher
    ros::Publisher ack_pub, data_pub, test_pub;

    // ros subscribers
    ros::Subscriber data_sub, test_sub;

    // acknowledge messages
    gps_testing::Ack acknowledge;
    gps_testing::Move rssi;
    std_msgs::UInt8MultiArray rdata;

    // private variables
    uint8_t count;
    uint8_t stuck_count;
    uint8_t full;
    uint8_t seq;
    uint8_t bytes;
    bool at_enable, at_request;
    bool robot;
    bool rf_test;
    bool ack_data;
    bool first;
};
#endif // acknowledge
