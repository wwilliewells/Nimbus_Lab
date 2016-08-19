/*
 * FILE: xbee.h
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

#ifndef _XBEE_H_
#define _XBEE_H_

// ros includes
#include "ros/ros.h"
#include <std_msgs/UInt8MultiArray.h>

// package message include
#include <gps_testing/Bytes.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
#define STATE_START 0
#define STATE_LEN0 1
#define STATE_LEN1 2
#define STATE_DATA 3
#define STATE_CHECKSUM 4
#define MAX_DATA_LEN 128

// class
class XBee{
  public:
    // con(de)structors
    XBee();
    ~XBee();
  private:
    void callbackData(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);
    void callbackDevice(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);
    void stream(const ros::TimerEvent&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    void initTimers();
     
    // ros objects
    ros::NodeHandle nh, pnh;
    // ros publisher
    ros::Publisher byte_pub, rx_pub, tx_pub;
    // ros subscribers
    ros::Subscriber rx_sub, device_sub;
    ros::Timer dataStream;

    // Mutual Exclusion Lock
    boost::mutex mel;

    // acknowledge messages
    gps_testing::Bytes byteLog;
    std_msgs::UInt8MultiArray streamData, dataRx, deviceTx;

    // in(ex)ternal parameters
    uint32_t bytes_received;
    uint32_t total_bytes;
    uint8_t seq;
    uint8_t d_seq;
    uint8_t bytesRead;
    uint8_t c;
    uint16_t length;
    int rcvr;
    int j;
};
#endif
