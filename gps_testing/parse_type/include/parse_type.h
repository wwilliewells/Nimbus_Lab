/*
 * File: parse_type.h
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

#ifndef _PARSE_TYPE_H_
#define _PARSE_TYPE_H_

// ros includes
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

// package message include
#include <gps_testing/Rssi.h> 

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
//#define PI

// class
class parse_type{
  public:
    // con(de)structors
    parse_type();
    ~parse_type();
  private:
    void callbackType(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // ros objects
    ros::NodeHandle nh, pnh;
    // ros publisher
    ros::Publisher rssi_pub, device_pub;
    // ros subscribers
    ros::Subscriber rx_sub;

    // class messages
    std_msgs::UInt8MultiArray device_data;
    gps_testing::Rssi rssi_data;

    // in(ex)ternal parameters
};
#endif // parse_type
