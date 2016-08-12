/*
 * FILE: sphere.cpp
 * AUTHOR: Wiliiam Willie Wells
 * DATE: April - May 2015
 */

// class include
#include "../include/sphere/sphere.h"

// defines
#define ROS_PKG "sphere"
#define ROS_NODE "sphere"

// constructor
Sphere::Sphere():nh(),pnh("~"){
    // initialize parameters
    initParams();

    // initialize publishers
    initPublishers();

    // initialize subscribers
    initSubscribers();

    // initialize services
    initServices();

    // start main thread
    startThread(1);
}

// destructor
Sphere::~Sphere(){
    stopThread(1);
}

// Main
int main(int argc, char **argv){
    // initialize ros node handle
    ros::init(argc,argv,ROS_NODE);

    // log start
    ROS_INFO("Started node %s", ROS_NODE);

    // create and start sphere instance
    Sphere sfear;

    // keep spinning
    ros::spin();

    // log stop
    ROS_INFO("Stoped node %s", ROS_NODE);

    return 0;
}
