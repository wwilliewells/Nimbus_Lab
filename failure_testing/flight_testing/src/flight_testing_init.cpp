/*
 * FILE: flight_testing_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2015
 */

// c++ include
#include <cmath>

// class include
#include "../include/flight_testing.h"

// initialize parameters
void FlightTest::initParams(){
    // initialize private booleans
    first = launch_request = start_request =true;
    last = land_request = land_idle = test_enabled = false;
    publish_change = change_pose = change_state = partial = false;
    control_received = true;

    // set launch height
    pnh.param("launch_height",launchZ,0.7);
    if(launchZ < 0.1 || launchZ > 1.4){ launchZ = 0.7; }

    // set test position change variance used in x, y, and z
    pnh.param("position_change",poseChange,0.2);
    if(fabs(poseChange) > 1.0){ poseChange = 0.5; }
    ROS_INFO("Z:%f, change:%f",launchZ,poseChange);

    // set relative length of each pre launch test
    pnh.param("test_length", test_length, 16);
    if(test_length < 8){ test_length = 16; }

    // set control rate
    pnh.param("ctrl_rate",ctrl_rate,25);
    if(ctrl_rate < 10 || ctrl_rate > 200){ ctrl_rate = 25; }
    
    // set arducopter
    pnh.param("arducopter",arducopter,false);

    // initialize private real numbers
    count = 0;
    move = 1.0;
    test_command = 0;
    initialX = 1.9;
    initialY = -1.9;
}

// initialize subscribers
void FlightTest::initSubscribers(){
  // subscribe to position
  pose_sub = nh.subscribe<collab_msgs::SubjectPose>("subject_pose",1,&FlightTest::callbackPose,this);

  // subscribe to state
  state_sub = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",1,&FlightTest::callbackState,this);

  // subscribe to status
  status_sub = nh.subscribe<collab_msgs::SubjectStatus>("subject_status",1,&FlightTest::callbackStatus,this);
		
  // subscribe to quad control input
  control_sub = nh.subscribe<collab_msgs::QuadCtrlInput>("quad_ctrl_input",1,&FlightTest::callbackControlInput,this);

  // subscribe to arducopter ready communication
  arducopter_sub = nh.subscribe<flight_testing::Arducopter>("arducopter",1,&FlightTest::callbackArducopter,this);
}

// initialize publishers
void FlightTest::initPublishers(){
    // publish position
    pose_pub = nh.advertise<collab_msgs::SubjectPose>("task_waypose",1,true);

    // publish state
    state_pub = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state",1,true);

    // publish prelaunch test
    test_pub = nh.advertise<collab_msgs::QuadCtrlInput>("motor_test_input",1,true);
}

