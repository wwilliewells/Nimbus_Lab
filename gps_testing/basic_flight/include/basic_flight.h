/*
 * FILE: basic_flight.h
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

#ifndef _BASIC_FLIGHT_H_
#define _BASIC_FLIGHT_H_

// ros includes
#include "ros/ros.h"

// mit_asctec includes
#include <collab_msgs/SubjectCtrlState.h>
#include <collab_msgs/SubjectPose.h>
#include <collab_msgs/SubjectStatus.h>
#include <collab_msgs/QuadCtrlInput.h>

// package message include
#include <gps_testing/Flight.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines

// class
class BasicFlight{
  public:
    // con(de)structors
    BasicFlight();
    ~BasicFlight();

    //bool getLast();
  private:
    // callbacks
    void callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const>&);
    void callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const>&);
    void callbackStatus(const boost::shared_ptr<collab_msgs::SubjectStatus const>&);
    void callbackFlight(const boost::shared_ptr<gps_testing::Flight const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    //void initServices();

    // start thread
    //void startThread(int);

    // stop thread
    //void stopThread(int);

    // active threads
    //void basicFlightThread();
    //void stateThread();

    // state change
    void startup();
    void launch();
    void land();
    void changeState(double,int);

    // ros objects
    ros::NodeHandle nh, pnh;

    // ros publisher
    ros::Publisher state_pub;

    // ros subscribers
    ros::Subscriber pose_sub, state_sub, status_sub, flight_sub;

    // thread objects
    //boost::mutex mel;
    //boost::shared_ptr<boost::thread> state_thread, flight_thread;

    // position and state messages
    collab_msgs::SubjectPose current_pose;
    collab_msgs::SubjectCtrlState command_state, previous_state;
    collab_msgs::SubjectStatus status;
    gps_testing::Flight flight;

    // flags
    volatile bool status_received;
    volatile bool state_received;
    volatile bool pose_received;

    // launch file parameters
};
#endif // basic flight
