/*
 * FILE: test_flight.h
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

#ifndef _TEST_FLIGHT_H_
#define _TEST_FLIGHT_H_

// ros includes
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

// mit_asctec includes
#include <collab_msgs/SubjectCtrlState.h>
#include <collab_msgs/SubjectPose.h>

// package message include
#include <gps_testing/Flight.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
#define PI 3.1415926535

// class
class TestFlight{
  public:
    // con(de)structors
    TestFlight();
    ~TestFlight();

    bool getLast();
  private:
    // callbacks
    void callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const>&);
    void callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();

    // start thread
    void startThread(int);

    // stop thread
    void stopThread(int);

    // active threads
    void flightThread();
    void poseThread();
    void stateThread();

    // pre and post hover-move state change
    void basicFlight();

    // ros objects
    ros::NodeHandle nh, pnh;

    // ros publisher
    ros::Publisher pose_pub, state_pub, flight_pub;

    // ros subscribers
    ros::Subscriber pose_sub, state_sub;

    // thread objects
    boost::mutex mel;
    boost::shared_ptr<boost::thread> pose_thread, state_thread;
    boost::shared_ptr<boost::thread> flight_thread;

    // position and state messages
    collab_msgs::SubjectPose current_pose, target_pose;
    collab_msgs::SubjectCtrlState current_state, target_state;
    gps_testing::Flight basic_flight;

    // flags
    volatile bool change_state;
    volatile bool change_pose;
    volatile bool publish_change;
    volatile bool start_request;
    volatile bool launch_request;
    volatile bool land_request;
    volatile bool first;
    volatile bool last;
    volatile bool partial;

    // private variables
    double initialX, initialY;
    uint8_t count;

    // launch file parameters
    double launchZ;
    double poseChange;
    int ctrl_rate;  
};
#endif // test flight
 
