/*
 * FILE: parse_serial.h
 * AUTHOR: William Willie Wells, modified from code
 *         written by: Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2016
 */

#ifndef _PARSE_SERIAL_H_
#define _PARSE_SERIAL_H_

// ros includes
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

// c++ include
#include <boost/thread/thread.hpp>

// package message include
#include <gps_testing/Sensor.h>

// namespace
using namespace std;

// class
class ParseSerial{
  public:
    // con(de)structors
    ParseSerial();
    ~ParseSerial();

  private:
    // callbacks
    void callbackSerialRx(const boost::shared_ptr<std_msgs::UInt8MultiArray const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // main process
    void processByte();

    // ros objects
    ros::NodeHandle nh;

    // ros publisher
    ros::Publisher sensor, serial_tx;

    // ros subscribers
    ros::Subscriber serial_rx/*, flight, task*/;
       
    // thread object
    boost::mutex mel;

    // serial messages
    std_msgs::UInt8MultiArray message;
    gps_testing::Sensor sensor_value;
       
    // state enumeration
    enum states{NOT_SYN, // waiting for '\r'
                READY, // '\r' received waiting for '\n'
                SENSOR1, // '\n' received, waiting for first sensor data
               };
    volatile enum states state;

    // private variables
    int size;
    unsigned int c;
    uint16_t value;
};
#endif // parse serial

