/*
 * FILE: send_serial.h
 * AUTHOR: William Willie Wells
 * DATE:July 2016
 */

#ifndef _SEND_SERIAL_H_
#define _SEND_SERIAL_H_

// ros includes
#include "ros/ros.h"
#include <std_msgs/UInt8MultiArray.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// class -- send a command to laser
class sendSerial{
  public:
    // con(de)structors
    sendSerial();
    ~sendSerial();

  private:
    // initializers
    void initPublishers();
    void initTimers();

    void send(const ros::TimerEvent&);

    // ros objects
    ros::NodeHandle nh;

    // ros publisher
    ros::Publisher data_pub;

    // ros timer
    ros::Timer timer;

    // messages
    std_msgs::UInt8MultiArray stream;
};
#endif // sendSerial`

