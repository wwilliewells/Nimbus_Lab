/*
 * FILE: parse_serial.cpp
 * AUTHOR: William Willie Wells, modified from code
 *         written by: Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2015
 */

// class include
#include "../include/parse_serial.h"

// defines
#define ROS_PKG "flightTest"
#define ROS_NODE "parseSerial"

// constructor
ParseSerial::ParseSerial():nh(){
    // initialize parameters
    initParams();

    // initialize publishers
    initPublishers();

    // initialize subscribers
    initSubscribers();
}

// destructor
ParseSerial::~ParseSerial(){}

// c++ Main
int main(int argc, char **argv){
    // initialize ros node handle
    ros::init(argc,argv,ROS_NODE);

    // log start
    ROS_INFO("Started node %s",ROS_NODE);

    // create and start parse serial instance
    ParseSerial passCereal;

    // keep spinning
    ros::spin();

    // log stop
    ROS_INFO("Stopped node %s", ROS_NODE);

    return 0;
}
