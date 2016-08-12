/*
 * FILE: parse_serial.h
 * AUTHOR: William Willie Wells, modified from code
 *         written by: Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2015
 */

#ifndef _PARSE_SERIAL_H_
#define _PARSE_SERIAL_H_

// ros includes
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

// mit_asctec includes
#include <collab_msgs/SubjectCtrlState.h>
#include <collab_msgs/SubjectPose.h>

// c++ include
#include <boost/thread/thread.hpp>

// package message include
#include <flight_testing/Sensor.h>

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
        void callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const>&);
        void callbackTask(const boost::shared_ptr<collab_msgs::SubjectPose const>&);

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
        ros::Subscriber serial_rx, flight, task;
       
        // thread object
        boost::mutex mel;

        // serial messages
        std_msgs::UInt8MultiArray message;
        std_msgs::Float64MultiArray float_message;
        collab_msgs::SubjectCtrlState current_state;
	collab_msgs::SubjectPose current_task;
        flight_testing::Sensor sensor_value;
       
        // state enumeration
        enum states{NOT_SYN, // waiting for '\r'
            READY, // '\r' received waiting for '\n'
            SENSOR1, // '\n' received, waiting for first sensor data
            SENSOR2, // previous sensor's data received, waiting for next sensor's data
            SENSOR3, // previous sensor's data received, waiting for next sensor's data
            SENSOR4, // previous sensor's data received, waiting for next sensor's data
        };
        volatile enum states state;

        // private variables
        bool decimal;
        //bool state_received;
        //bool task_received;
        volatile bool flying;
        int size;
        int place;
        unsigned int c;
        double value;
};
#endif // parse serial

