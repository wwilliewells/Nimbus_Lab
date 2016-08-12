/*
 * FILE: flight_testing_thread.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2015
 */

// class include
#include "../include/flight_testing.h" 

// unlock mutual exclusion to publish, sleep, and relock
void FlightTest::changeState(double wait_time,int state){
  command_state.state = state;
  mel.unlock();
  ros::Duration(wait_time).sleep();
  mel.lock();
}

// startup UAV
void FlightTest::startup(){
  mel.lock();

  if(previous_state.state == 0){
    // starting: do not need to land
    land_request = false;

    // transition to OFF
    while(previous_state.state != 1){ changeState(1.0,1); }
    
    // transition to STARTUP
    while(status.motors_status == 0){ if(previous_state.state == 2){ break; } changeState(3.0,2); }

    // force transition to idle state		
    while(previous_state.state != 4){ changeState(1.0,4); }
  }

  test_enabled = true; ROS_INFO("pitch and roll test enabled");
  start_request = false;

  mel.unlock();
}

// initial quad control input
void FlightTest::initialQCI(){
  target_qci.pitch = target_qci.roll = target_qci.yaw = 0.0;
  target_qci.thrust = -1.0;
  target_qci.pitch_ctrl = target_qci.roll_ctrl = target_qci.yaw_ctrl = target_qci.thrust_ctrl = 1;
  target_qci.altitude_ctrl = target_qci.gps_ctrl = 0;
}

// launch UAV
void FlightTest::launch(){
  mel.lock();

  // transition to LAUNCH
  target_pose.translation.x = initialX;
  target_pose.translation.y = initialY;
  target_pose.translation.z = launchZ;
  target_pose.rotation.z = 0.0;
  while(!control.gps_ctrl){ ROS_INFO("attempting launch"); changeState(2.0,5); }

  // success, launched
  if(control.gps_ctrl){ launch_request = false; ROS_INFO("Launched"); }

  mel.unlock();
}

// land UAV
void FlightTest::land(){
  ROS_INFO("landing");

  // land 
  mel.lock();

  // transition to HOVER
  changeState(1.0,7);
  while(previous_state.state == 8){ changeState(1.0,7); ROS_INFO("forced hover"); }

  // transition to LAND
  while(current_pose.translation.z > 0.06){ ROS_INFO("attempting descent"); changeState(3.0,6); }
		
  // transition to IDLE
  while(previous_state.state != 4){ 
    ROS_INFO("returning to idle"); changeState(2.0,4); land_idle = true; }
  //land_idle = true;

  // transition to OFF
  while(status.motors_status){ if(previous_state.state == 3){ break; } changeState(3.0,3); }
  land_idle = false;
    
  // transition to ESTOP
  while(previous_state.state != 0){ changeState(1.0,0); }

  // landing a success
  land_request = false;
  mel.unlock();
}

// stop a thread
void FlightTest::stopThread(int t){
  switch(t){
    case(0): flight_thread->join(); break;
    case(1): pose_thread->join(); break;
    case(2): state_thread->join(); break;
    //case(3): status_thread->join(); break;
    case(3): control_thread->join(); break;
    default:  break;
  }
}

// start a thread 
void FlightTest::startThread(int t){
  switch(t){
    case(0):
      flight_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&FlightTest::flightTestThread,this))); break;
    case(1):
      pose_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&FlightTest::poseThread,this))); break;
    case(2):
      state_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&FlightTest::stateThread,this))); break; 
    //case(3):
    //  status_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&FlightTest::statusThread,this))); break;
    case(3):
      control_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&FlightTest::controlThread,this))); break;
    default: break;
  }
}

// control thread
void FlightTest::controlThread(){
  // set publishing rate
  ros::Rate pub_rate(ctrl_rate);

  // publish custom control input
  while(ros::ok()){
    if(test_enabled || control_received){
      mel.lock();
			
      // iniital quad control inputs
      if(first){ initialQCI(); test_command = 0; }

      // attempt to abolish time travel
      if(test_enabled){ 
        // y positive, y negative, x positive, x negative
        if(test_command < test_length){
          target_qci.pitch = -move;
          target_qci.roll = target_qci.yaw = 0.0;
        }else if(test_command < 2*test_length && test_command > 1*test_length-1){
          target_qci.pitch = move;
          target_qci.roll = target_qci.yaw = 0.0;
        }else if(test_command < 3*test_length && test_command > 2*test_length-1){
          target_qci.roll = -move;
          target_qci.pitch = target_qci.yaw = 0.0;
        }else if(test_command < 4*test_length && test_command > 3*test_length-1){
          target_qci.roll = move;
          target_qci.pitch = target_qci.yaw = 0.0;
        }else if(test_command > 4*test_length){ 
          target_qci.roll = target_qci.pitch = target_qci.yaw = 0.0; 
          test_enabled = false; ROS_INFO("exitting pitch and roll test"); 
        }
        test_command++;
      }else if(land_idle && previous_state.state != 3){ initialQCI(); }
      else{ target_qci = control; }

      // synchronize time
      target_qci.header.stamp = ros::Time::now();
	
      // publish quad control input
      test_pub.publish(target_qci);
	
      // reset publishing variable		
      control_received = false;
			
      mel.unlock();
    }
    pub_rate.sleep();
  }
}

// position thread
void FlightTest::poseThread(){
  // set publishing rate
  ros::Rate pub_rate(ctrl_rate);

  // publish position data
  while(ros::ok()){
    if(publish_change || first){
      mel.lock();
      target_pose.header.stamp = ros::Time::now();
      //ROS_INFO("X:%f, Y:%f, Z:%f, W:%f,C:%i",target_pose.translation.x,target_pose.translation.y,target_pose.translation.z,target_pose.rotation.z,count);
      // publish new position
      pose_pub.publish(target_pose);
      change_pose = false;

      // reset publishing condition
      if(partial){ publish_change = false; partial = false; }
      else{ partial = true; }

      mel.unlock();
    }
    pub_rate.sleep();
  }
}

// state thread
void FlightTest::stateThread(){
  // set publishing rate
  ros::Rate pub_rate(ctrl_rate);

  // publish state data
  while(ros::ok()){
    if(publish_change){
      mel.lock();
      command_state.header.stamp = ros::Time::now();

      // iterate through target positions based on being in moving state
      if(command_state.state == 8){ count++; }

      // publish new state
      state_pub.publish(command_state);
      change_state = false;

      // reset publishing condition
      if(partial){ publish_change = false; partial = false; }
      else{ partial = true; }

      // unlock and sleep
      mel.unlock();
    }
    pub_rate.sleep();
  }
}
       
// main thread
void FlightTest::flightTestThread(){
  // start pose, and state threads
  startThread(3);
  startThread(2);
  startThread(1);

  // main thread launchs, relies on callbacks, lands when count reachs 17
  while(ros::ok() && !last){
    // launch UAV
    if(launch_request){ 
      if(start_request){ startup(); ROS_INFO("startup complete"); }
      if(test_enabled == false /*&& arducopter == false*/){ launch(); } 
    }
        
    // condition to land
    if(count > 16){ land_request = true; }

    // land UAV
    if(land_request){ land(); last = true; }
  }
    
  // stop state, and pose threads
  stopThread(1);
  stopThread(2);
  stopThread(3);
}

