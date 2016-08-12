/*1
 * FILE: gps_testing_thread.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/gps_testing.h" 

// publish a request to startup, launch, or land handled by node, basic_flight
void GpsTest::basicFlight(){
  basic_flight.header.stamp = ros::Time::now();
  flight_pub.publish(basic_flight);
}

// stop a thread
void GpsTest::stopThread(int t){
  switch(t){
    case(0): flight_thread->join(); break;
    case(1): pose_thread->join(); break;
    case(2): state_thread->join(); break;
    //case(3): status_thread->join(); break;
    default: break;
  }
}

// start a thread 
void GpsTest::startThread(int t){
  switch(t){
    case(0):
      flight_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&GpsTest::gpsTestThread,this))); break;
    case(1):
      pose_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&GpsTest::poseThread,this))); break;
    case(2):
      state_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&GpsTest::stateThread,this))); break; 
    default: break;
  }
}

// position thread
void GpsTest::poseThread(){
  // set publishing rate
  ros::Rate pub_rate(ctrl_rate);

  // publish position data
  while(ros::ok()){
    if(publish_change || first){
      mel.lock();
      target_pose.header.stamp = ros::Time::now();
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
void GpsTest::stateThread(){
  // set publishing rate
  ros::Rate pub_rate(ctrl_rate);

  // publish state data
  while(ros::ok()){
    if(publish_change){
      mel.lock();
      target_state.header.stamp = ros::Time::now();

      // publish new state
      state_pub.publish(target_state);
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
void GpsTest::gpsTestThread(){
  // start pose, and state threads
  startThread(2);
  startThread(1);

  // main thread launchs, relies on callbacks, lands when a land request is received
  while(ros::ok() && !last){
    // launch UAV
    if(launch_request){ 
      if(start_request){
        basic_flight.startup = true;
        basic_flight.launch = basic_flight.land = false;
        basicFlight();
        start_request = false; 
        ROS_INFO("startup complete"); 
      }
      else{ 
        basic_flight.launch = true;
        basic_flight.startup = basic_flight.land = false;
        basicFlight();
        launch_request = false;
      } 
    }

    // land UAV
    if(land_request){ 
      basic_flight.land = true;
      basic_flight.launch = basic_flight.startup = false;
      basicFlight();
      land_request = false;
    }
  }
    
  // stop state, and pose threads
  stopThread(1);
  stopThread(2);
}

