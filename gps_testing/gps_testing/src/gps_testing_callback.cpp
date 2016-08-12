/*
 * FILE: gps_testing_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/gps_testing.h" 

// callback for state change
void GpsTest::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
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
void GpsTest::callbackPose(const boost::shared_ptr<collab_msgs::SubjectPose const> &msg){
  current_pose = (*msg);

  // set target position if not finished
  if(!publish_change && !change_pose && !last){
    // set rotation
    target_pose.rotation.z = PI; // N = 0.0, E = PI/2, S = PI, W = 3PI/2

    // set initial position 
    if(first && origin_received){ 
      initialX = target_pose.translation.x = origin_gps.latitude; 
      initialY = target_pose.translation.y = origin_gps.longitude; 
      target_pose.translation.z = 0.0;
      first = false; 
    }
    else if(origin_received){
      if(!land_request && current_pose.translation.z > 0.1){ 
        target_pose.translation.z = current_pose.translation.z;
      }
      else if(!land_request){ target_pose.translation.z = launchZ; }
      else{ target_pose.translation.z = 0.0; }
      target_pose.translation.x = current_pose.translation.x; 
      target_pose.translation.y = current_pose.translation.y;
    }

    // change position
    if(gps_test){ // NS-HP: 10km, >38db/Hz
      if(changeLat){ target_pose.translation.x += poseChange; }
      else{ target_pose.translation.y += poseChange; }
      gps_test = false; rf_test = true;
    }/*laser rated at 10m altitude*/
    else if(rf_test && current_pose.translation.z < 10.0 + zChange){ 
      target_pose.translation.z += zChange;
    } // descend: could make incremental
    else{ rf_test = false; target_pose.translation.z = launchZ; }

    // if near launch altitude move to next location
    //if(!rf_test && !gps_test && current_pose.translation.z < launchZ + 0.5){ gps_test = true; }

    // publishing condition
    change_pose = true;
  }

  // confirm target with land target
  if(land_request){
    target_pose.translation.x = initialX;
    target_pose.translation.y = initialY;
  }
}

// callback for gps origin
void GpsTest::callbackGpsOrigin(const boost::shared_ptr<collab_msgs::SubjectGps const> &msg){
  origin_gps = (*msg); origin_received = true;
}

// callback for laser repositioned 
void GpsTest::callbackMove(const boost::shared_ptr<gps_testing::Move const> &msg){
  if(msg->enable_gps_test){ gps_test = true; }
  if(msg->enable_rf_test){ rf_test = true; }
  if(msg->end_test){ land_request = true; }
}

