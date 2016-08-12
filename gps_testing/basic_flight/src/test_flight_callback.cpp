/*
 * FILE: test_flight_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/test_flight.h" 

// callback for state change
void TestFlight::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
  current_state = (*msg);
  // set target state if a target position is available and not finished
  if(!publish_change && !change_state && !last){
    // alternate between moving and hovering
    if(!land_request){
      if(current_state.state == 7){ target_state.state = 8; change_state = true; } 
      else if(current_state.state == 8){ target_state.state = 7; change_state = true; }   
    }
  }
  // publish if target state and target position obtain
  if(change_state && change_pose){ publish_change = true; }
}

// callback for position change
void TestFlight::callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const> &msg){
  current_pose = (*msg);

  // set target position if not finished
  if(!publish_change && !change_pose && !last){
    // set rotation
    target_pose.rotation.z = PI; // N = 0.0, E = PI/2, S = PI, W = 3PI/2

    // set initial position 
    if(first){ 
      initialX = target_pose.translation.x = current_pose.translation.x; 
      initialY = target_pose.translation.y = current_pose.translation.y; 
      target_pose.translation.z = 0.0;
      first = false; 
    }
    else{
      if(!land_request && current_pose.translation.z > 0.1){ 
        target_pose.translation.z = current_pose.translation.z;
      }
      else if(!land_request){ target_pose.translation.z = launchZ; }
      else{ target_pose.translation.z = 0.0; }
      target_pose.translation.x = initialX; 
      target_pose.translation.y = initialY;
    }

    // change position
    if(count == 1){ target_pose.translation.x += poseChange; }
    else if(count == 2){ target_pose.translation.y += poseChange; }
    else if(count == 3){ target_pose.translation.z += poseChange; }
    else if(count == 4){ target_pose.rotation.z = 0.0; }

    // publishing condition
    change_pose = true;
  }

  // confirm target with land target
  if(land_request){
    target_pose.translation.x = initialX;
    target_pose.translation.y = initialY;
    target_pose.translation.z = 0.0;
  }
}

