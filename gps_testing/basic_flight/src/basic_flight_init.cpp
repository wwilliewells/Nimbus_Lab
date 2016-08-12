/*
 * FILE: basic_flight_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/basic_flight.h"

// initialize parameters
void BasicFlight::initParams(){ pose_received = state_received = status_received = false; }

// initialize subscribers
void BasicFlight::initSubscribers(){
  // subscribe to position
  pose_sub = nh.subscribe<collab_msgs::SubjectPose>("subject_pose",1,&BasicFlight::callbackPose,this);

  // subscribe to state
  state_sub = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",1,&BasicFlight::callbackState,this);

  // subscribe to status
  status_sub = nh.subscribe<collab_msgs::SubjectStatus>("subject_status",1,&BasicFlight::callbackStatus,this);

  // subscribe to state change message
  flight_sub = nh.subscribe<gps_testing::Flight>("flight",1,&BasicFlight::callbackFlight,this);
}

// initialize publishers
void BasicFlight::initPublishers(){
  // publish state
  state_pub = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state",1,true);
}

