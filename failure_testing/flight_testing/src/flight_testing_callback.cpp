/*
 * FILE: flight_testing_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2015
 */

// class include
#include "../include/flight_testing.h" 

// callback for state change
void FlightTest::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
  //mel.lock();
  
  previous_state = (*msg);
  // set target state if a target position is available and not finished
  if(!publish_change && !change_state && !last){
    // between moving and hovering
    if(!land_request && count < 18){
      if(previous_state.state == 7){ command_state.state = 8; } 
      else if(previous_state.state == 8){ command_state.state = 7; }
    }

    // change state
    if(command_state.state == 0){ change_state = true; land_request = false; }
    else if(command_state.state == 1){ if(previous_state.state == 0){ change_state = true; } }
    else if(command_state.state == 2){ if(previous_state.state == 1){ change_state = true; } }
    else if(command_state.state == 3){ if(previous_state.state == 4){ change_state = true; } }
    else if(command_state.state == 4){ if(previous_state.state == 6){ change_state = true; } }
    else if(command_state.state == 5){ if(previous_state.state == 4){ change_state = true; } }
    else if(command_state.state == 6){ if(previous_state.state == 7){ change_state = true; } }
    else if(command_state.state == 7){ if(previous_state.state == 8){ change_state = true; } }
    else if(command_state.state == 8){ if(previous_state.state == 7){ change_state = true; } }
        
  }
  // publish if target state and target position obtain
  if(change_state && change_pose){ publish_change = true; }

  //mel.unlock();
}

// callback for position change
void FlightTest::callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const> &msg){
  //mel.lock();
  current_pose = (*msg);

  // set target position if not finished
  if(!publish_change && !change_pose && !last){
    ROS_INFO("manuever: %i",count);// set rotation
    if(previous_state.state < 7){ target_pose.rotation.z = PI; }
    else{
      if(count == 1 || count == 4 || count == 15){ target_pose.rotation.z =  PI/2; }
      else if(count == 2 || count == 7){ target_pose.rotation.z = 3*PI/2; } 
      else if(count == 3 || count == 10){ target_pose.rotation.z = PI; }
      else{ target_pose.rotation.z = 0.0; }
    }

    // set initial position 
    if(first){ 
      initialX = target_pose.translation.x = current_pose.translation.x; 
      initialY = target_pose.translation.y = current_pose.translation.y; 
      //target_pose.translation.y = current_pose.translation.y;
      target_pose.translation.z = 0.0;
      first = false; }
    else{
      target_pose.translation.z = launchZ;
      target_pose.translation.x = initialX; 
      target_pose.translation.y = initialY;
    }

    // change position
    if(count == 4){ target_pose.translation.x += poseChange; }
    else if(count == 5){ target_pose.translation.x -= poseChange; }
    else if(count == 7){ target_pose.translation.y -= poseChange; }
    else if(count == 8){ target_pose.translation.y += poseChange; }
    else if(count == 10){ target_pose.translation.z += poseChange; }
    else if(count == 12){ // change in x and y
      target_pose.translation.x += poseChange;
      target_pose.translation.y += poseChange; }
    else if(count == 13){ // change in x and z
      target_pose.translation.y += poseChange;
      target_pose.translation.z += poseChange; }
    else if(count == 15){ // count = 14 change in y and z; change in x, y, and w 
      target_pose.translation.x -= poseChange;
      target_pose.translation.y -= poseChange; }
    else if(count == 16){ target_pose.translation.z += poseChange; } // change in (x,y,z,w)
    else if(count > 16){ land_request = true; } // count = 17 change in z

    // publishing condition
    change_pose = true;
  }

  // confirm target with land target
  if(land_request == true || (launch_request == true && !first)){
    target_pose.translation.x = initialX;
    target_pose.translation.y = initialY;
    // enforce requested height on launch and land
    if(land_request == true && previous_state.state < 7){ target_pose.translation.z = 0.0; }
    else if(launch_request == true){ 
      if(previous_state.state >= 2){ target_pose.translation.z = launchZ; }
      else{ target_pose.translation.z = 0.0; }
    }
    // enforce requested rotation on launch and land
    if(previous_state.state == 6 || previous_state.state == 2){ target_pose.rotation.z = PI; }
    else if(land_request == true){ target_pose.rotation.z = 0.0; }
  }  
        
  //mel.unlock();
}

// callback for status
void FlightTest::callbackStatus(const boost::shared_ptr<collab_msgs::SubjectStatus const> &msg){
  status = (*msg); //status_received = true;
}

// callback for quad control input
void FlightTest::callbackControlInput(const boost::shared_ptr<collab_msgs::QuadCtrlInput const> &msg){
  control = (*msg);
  control_received = true;
}

void FlightTest::callbackArducopter(const boost::shared_ptr<flight_testing::Arducopter const> &msg){
  arducopter = msg->busy;
}

