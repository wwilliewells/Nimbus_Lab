/*
 * FILE: gps_device.h
 * AUTHOR: William Willie Wells
 * DATE:June 2016
 */

#ifndef _GPS_DEVICE_H_
#define _GPS_DEVICE_H_

// ros includes
#include "ros/ros.h"
#include <std_msgs/UInt8MultiArray.h>

// package message include

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// class -- receive/transmit with respect to rf
class gpsDevice{
  public:
    // con(de)structors
    gpsDevice();
    ~gpsDevice();

  private:
    // callbacks
    void callbackReceive(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);
    void callbackTransmit(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // ros objects
    ros::NodeHandle nh, pnh;

    // ros publisher
    ros::Publisher transmit_pub, receive_pub;

    // ros subscribers
    ros::Subscriber transmit_sub, receive_sub;

    // messages
    std_msgs::UInt8MultiArray tx_data, rx_data;

    // private variables
    int device;
    uint8_t decode;
    bool radio;
};
#endif // gpsDevice

