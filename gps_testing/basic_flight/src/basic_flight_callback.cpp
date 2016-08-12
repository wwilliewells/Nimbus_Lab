/*
 * FILE: basic_flight_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/basic_flight.h" 

// reset message received state, publish commanded state, sleep
void BasicFlight::changeState(double wait_time,int state){
  state_received = status_received = pose_received = false;
  command_state.state = state;
  command_state.header.stamp = ros::Time::now();

  // publish new state
  state_pub.publish(command_state);
  ros::Duration(wait_time).sleep();  
}

// startup UAV
void BasicFlight::startup(){
  if(previous_state.state == 0){
    // transition to OFF
    while(previous_state.state != 1){ changeState(1.0,1); }

    // transition to STARTUP
    while(status.motors_status == 0){ if(previous_state.state == 2){ break; } changeState(3.0,2); }

    // force transition to idle state           
    while(previous_state.state != 4){ changeState(1.0,4); }
  }
}

// launch UAV
void BasicFlight::launch(){
  if(previous_state.state == 4){
    // transition to LAUNCH
    while(previous_state.state < 5){ ROS_INFO("attempting launch"); changeState(2.0,5); }

    // success, launched
    ROS_INFO("Launched");
  }
}

// land UAV
void BasicFlight::land(){
  if(previous_state.state == 7 || previous_state.state == 8){
    ROS_INFO("landing");

    // transition to HOVER
    changeState(1.0,7);
    while(previous_state.state == 8){ changeState(1.0,7); ROS_INFO("forced hover"); }

    // transition to LAND
    while(current_pose.translation.z > 0.06){ ROS_INFO("attempting descent"); changeState(3.0,6); }

    // transition to IDLE
    while(previous_state.state != 4){ ROS_INFO("returning to idle"); changeState(2.0,4); }

    // transition to OFF
    while(status.motors_status){ if(previous_state.state == 3){ break; } changeState(3.0,3); }

    // transition to ESTOP
    while(previous_state.state != 0){ changeState(1.0,0); }
  }
}

// callback for state change
void BasicFlight::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
  previous_state = (*msg); state_received = true;
}

// callback for position change
void BasicFlight::callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const> &msg){
  current_pose = (*msg); pose_received = true;
}

// callback for status
void BasicFlight::callbackStatus(const boost::shared_ptr<collab_msgs::SubjectStatus const> &msg){
  status = (*msg); status_received = true;
}

// callback for quad control input
void BasicFlight::callbackFlight(const boost::shared_ptr<gps_testing::Flight const> &msg){
  flight = (*msg);
  if(flight.startup && state_received && status_received){ startup(); }
  else if(flight.launch && state_received){ launch(); }
  else if(flight.land && state_received && status_received && pose_received){ land(); }
}

