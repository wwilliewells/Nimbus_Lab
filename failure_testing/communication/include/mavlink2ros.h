/*
 * FILE: mavlink2ros.h
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

#ifndef _MAVLINK2ROS_H_
#define _MAVLINK2ROS_H_

// ros includes
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt8.h"

// c++ include
#include <boost/thread/thread.hpp>

// package message include
#include <flight_testing/Mavlink.h>

// namespace
using namespace std;

// class
class mrTelemetry{
  public:
    // con(de)structors
    mrTelemetry();
    ~mrTelemetry();

  private:
    // callbacks: from, to
    void callbackRobotRx(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);
    void callbackMavlinkTx(const boost::shared_ptr<flight_testing::Mavlink const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // main process
    void processByte();

    // ros objects
    ros::NodeHandle nh;

    // ros publisher
    ros::Publisher robot_transmit, mavlink_receive;

    // ros subscribers
    ros::Subscriber robot_receive, mavlink_transmit;
       
    // thread object
    boost::mutex mel;

    // serial messages
    std_msgs::UInt8MultiArray message_rx,message_tx;
    flight_testing::Mavlink mavmsg_rx,mavmsg_tx;
     
    // state enumeration
    enum states{
      START, // 0xFE
      PAYLOAD_LEN, // 
      SEQUENCE, // 
      SYSTEM_ID, // 
      COMPONENT_ID,
      MESSAGE_ID,
      PAYLOAD,
      CRC
    };
    volatile enum states state;

    // private variables
    bool switch_state;
    bool first;
    int size; // byte stream size
    uint8_t bytes; // cyclic redundancy check size
    int tmp; // as a byte count for payload and crc
    uint8_t c; // byte in stream
    uint16_t crc; // crc
    uint8_t seq;
};
#endif // mavlink 2 ros

