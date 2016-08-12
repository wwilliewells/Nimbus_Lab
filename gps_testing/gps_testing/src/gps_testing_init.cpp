/*
 * FILE: gps_testing_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// c++ include
#include <cmath>

// class include
#include "../include/gps_testing.h"

// initialize parameters
void GpsTest::initParams(){
  // initialize private booleans
  first = gps_test = launch_request = start_request =  true;
  last = rf_test = land_request = origin_received = false;
  publish_change = change_state = change_pose = partial = false;

  // set launch height
  pnh.param("launch_height",launchZ,0.7);
  if(launchZ < 0.1 || launchZ > 1.4){ launchZ = 0.7; }

  // set test position change variance used in x
  pnh.param("position_change",poseChange,50.0);
  if(fabs(poseChange) > 2000.0){ poseChange = 50.0; }
  poseChange *= 0.3048; // feet to meters
  ROS_INFO("Z:%f, change:%f",launchZ,poseChange);

  // set test position change variance used in x
  pnh.param("altitude_change",zChange,0.7);
  if(fabs(zChange) > 2.0){ zChange = 0.7; }

  // set control rate
  pnh.param("ctrl_rate",ctrl_rate,25);
  if(ctrl_rate < 10 || ctrl_rate > 200){ ctrl_rate = 25; }

  // set latitude or longitude change
  pnh.param("lat_or_long",changeLat,true);
}

// initialize subscribers
void GpsTest::initSubscribers(){
  // subscribe to position
  pose_sub = nh.subscribe<collab_msgs::SubjectPose>("subject_pose",1,&GpsTest::callbackPose,this);

  // subscribe to state
  state_sub = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",1,&GpsTest::callbackState,this);

  // subscribe to gps origin
  origin_sub = nh.subscribe<collab_msgs::SubjectGps>("gps_origin",10,&GpsTest::callbackGpsOrigin,this);

  // subscribe to resume test topic
  test_sub = nh.subscribe<gps_testing::Move>("resume_test",1,&GpsTest::callbackMove,this);
}

// initialize publishers
void GpsTest::initPublishers(){
  // publish position
  pose_pub = nh.advertise<collab_msgs::SubjectPose>("task_waypose",1,true);

  // publish state
  state_pub = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state",1,true);

  // publish prelaunch test
  flight_pub = nh.advertise<gps_testing::Flight>("flight",1,true);
}

