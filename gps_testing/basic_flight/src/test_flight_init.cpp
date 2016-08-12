/*
 * FILE: test_flight_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// c++ include
#include <cmath>

// class include
#include "../include/test_flight.h"

// initialize parameters
void TestFlight::initParams(){
  // initialize private booleans
  first = launch_request = start_request =  true;
  last = land_request = false;
  publish_change = change_state = change_pose = partial = false;

  // set launch height
  pnh.param("launch_height",launchZ,0.7);
  if(launchZ < 0.1 || launchZ > 1.4){ launchZ = 0.7; }

  // set test position change variance used in x
  pnh.param("position_change",poseChange,0.7);
  if(fabs(poseChange) > 1.0){ poseChange = 0.7; }
  ROS_INFO("Z:%f, change:%f",launchZ,poseChange);

  // set control rate
  pnh.param("ctrl_rate",ctrl_rate,25);
  if(ctrl_rate < 10 || ctrl_rate > 200){ ctrl_rate = 25; }

  count = 0;
}

// initialize subscribers
void TestFlight::initSubscribers(){
  // subscribe to position
  pose_sub = nh.subscribe<collab_msgs::SubjectPose>("subject_pose",1,&TestFlight::callbackPose,this);

  // subscribe to state
  state_sub = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",1,&TestFlight::callbackState,this);
}

// initialize publishers
void TestFlight::initPublishers(){
  // publish position
  pose_pub = nh.advertise<collab_msgs::SubjectPose>("task_waypose",1,true);

  // publish state
  state_pub = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state",1,true);

  // publish prelaunch test
  flight_pub = nh.advertise<gps_testing::Flight>("flight",1,true);
}

