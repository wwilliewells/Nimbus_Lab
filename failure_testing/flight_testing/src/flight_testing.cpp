/*
 * FILE: flight_testing.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2015 ...
 */

// class include
#include "../include/flight_testing.h"

// defines
#define ROS_PKG "flightTest"
#define ROS_NODE "flightTest"

// constructor
FlightTest::FlightTest():nh(),pnh("~"){
    // initialize parameters
    initParams();ROS_INFO("parameteers initialized");

    // initialize publishers
    initPublishers();ROS_INFO("publishers initialized");

    // initialize subscribers
    initSubscribers();ROS_INFO("subscribers initialized");

    // initialize services

    // start main thread
    startThread(0);
}

// destructor
FlightTest::~FlightTest(){ stopThread(0); }

// return true if landing is a success
bool FlightTest::getLast(){ return last; }

// c++ Main
int main(int argc, char **argv){
    // initialize ros node handle
    ros::init(argc,argv,ROS_NODE);

    // log start
    ROS_INFO("Started node %s",ROS_NODE);

    // create and start flight test instance
    FlightTest flytetest;
    
    // keep spinning until landing acheived
    while(!flytetest.getLast()){ ros::spinOnce(); }
    
    // log stop and shutdown
    ROS_INFO("Stopped node %s", ROS_NODE);
    ros::shutdown();

    return 0;
}
