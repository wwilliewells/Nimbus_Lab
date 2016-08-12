/*
 * FILE: flight_testing.h
 * AUTHOR: William Willie Wells
 * DATE: May 2015
 */

#ifndef _FLIGHT_TESTING_H_
#define _FLIGHT_TESTING_H_

// ros includes
#include "ros/ros.h"

// mit_asctec includes
#include <collab_msgs/SubjectCtrlState.h>
#include <collab_msgs/SubjectPose.h>
#include <collab_msgs/SubjectStatus.h>
#include <collab_msgs/QuadCtrlInput.h>

// package message include
#include <flight_testing/Arducopter.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
#define PI 3.1415926535

// class
class FlightTest{
  public:
    // con(de)structors
    FlightTest();
    ~FlightTest();

    bool getLast();
  private:
    // callbacks
    void callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const>&);
    void callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const>&);
    void callbackStatus(const boost::shared_ptr<collab_msgs::SubjectStatus const>&);
    void callbackControlInput(const boost::shared_ptr<collab_msgs::QuadCtrlInput const>&);
    void callbackArducopter(const boost::shared_ptr<flight_testing::Arducopter const>&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    //void initServices();

    // start thread
    void startThread(int);

    // stop thread
    void stopThread(int);

    // active threads
    void flightTestThread();
    void poseThread();
    void stateThread();
    void controlThread();

    // state change
    void startup();
    void initialQCI();
    void test();
    void launch();
    void land();
    void changeState(double,int);

    // ros objects
    ros::NodeHandle nh, pnh;

    // ros publisher
    ros::Publisher pose_pub, state_pub, test_pub;

    // ros subscribers
    ros::Subscriber arducopter_sub, control_sub, pose_sub, state_sub, status_sub;

    // thread objects
    boost::mutex mel;
    boost::shared_ptr<boost::thread> pose_thread, state_thread;
    boost::shared_ptr<boost::thread> control_thread, flight_thread;

    // position and state messages
    collab_msgs::SubjectPose current_pose, target_pose;
    collab_msgs::SubjectCtrlState command_state, previous_state;
    collab_msgs::QuadCtrlInput target_qci, control;
    collab_msgs::SubjectStatus status;
    //flight_testing::Arducopter busy;

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
    volatile bool control_received;
    volatile bool status_received;
    volatile bool test_enabled;
    volatile bool land_idle;
    bool arducopter;

    // private variables
    volatile int count, test_command;
    double initialX, initialY, move;

    // launch file parameters
    double launchZ;
    double poseChange;
    int test_length;
    int ctrl_rate;
};
#endif // flight test 
