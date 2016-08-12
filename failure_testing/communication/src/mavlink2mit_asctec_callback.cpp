/*
 * FILE: mavlink2mit_asctec_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2MIT_AscTec.h" 

// c++ include

uint8_t crc_seed[256] =
  {50,124,137,0,237,217,104,119,0,0,0,89,0,0,0,0,0,0,0,0,214,159,220,168,
  24,23,170,144,67,115,39,246,185,104,237,244,222,212,9,254,230,28,28,
  132,221,232,11,153,41,39,214,223,141,33,15,3,100,24,239,238,30,240,183,
  130,130,0,148,21,0,243,124,0,0,0,20,0,152,143,0,0,127,106,0,0,0,0,0,0,
  0,231,183,63,54,0,0,0,0,0,0,0,175,102,158,208,56,93,0,0,0,0,235,93,124,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,42,
  241,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,204,49,
  170,44,83,46,0};

// callback on mavlink message received
void mmaTranslation::callbackMavlinkRx(const boost::shared_ptr<flight_testing::Mavlink const> &msg){
  // obtain message
  mavmsg_rx = (*msg);
  mavmsg_received = true;
}

// callback for control inputs
void mmaTranslation::callbackControlInput(const boost::shared_ptr<collab_msgs::AsctecCtrlInput const> &msg){
  // obtain message
  rci_msg = (*msg);
  //ROS_INFO(" in control input callback");
  // supported cmds: 17,20,21,22,115,162,178,181,183,184,207,209,241, 242,
    // 17: changes mode but suppossed to be Loiter unlimited
    // 20: return to launch
    // 21: land
    // 22: takeoff
    // 115: condition_yaw
    // 178: do_change_speed
    // 181: do_set_relay
    // 183: do_set_servo -- TODO:if waypoints unavailable try this cmd
    // 184: do_repeat_servo
    // 209: mav_cmd_do_motor_test
    // 241: preflight_calibration
  
  //TODO: translate control inputs -- message id #39 mission request
  // mission item -- command 16, MAV_CMD_NAV_WAYPOINT
  /*Mission Param #1 Hold time in decimal seconds. (time to stay at MISSION for rotary wing)
    Mission Param #2 Acceptance radius in meters 
      (if the sphere with this radius is hit, the MISSION counts as reached)
    Mission Param #3 0 to pass through the WP, if > 0 radius in meters to pass by WP. 
      Positive value for clockwise orbit, negative value for counter-clockwise orbit. 
      Allows trajectory control.
    Mission Param #4	Desired yaw angle at MISSION (rotary wing)
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude*/
  if(mode > 128){
  mavmsg_ci.len = 18;
  mavmsg_ci.msgid = 70;
  pubGeneral(false,false,true);

  // payload 
  mavmsg_ci.payload.clear();
  for(int i=0;i<8;i++){ // TODO:scale input by 1000
    value_rc = 1500;
    //if(i==2){ value_f = 89.0; } // parameter 2
    //else if(i==2){ value_f = 0.0; } // parameter 3 
    // r: 1121 --> 1939, 1528 mid; p: 1089 --> 1907, 1500; t: 1117 --> 1929, 1494; 
    // y: 1114 --> 1938, 1487
    //else if(i==3){ value_f = 0.0/*1500 + ((float)(rci_msg.yaw)/2047.0)*500*/; } // parameter 4
    //else if(i==1){ value_f = 2.0/*1500 + ((float)(rci_msg.pitch)/2047.0)*500*/; } // parameter 5
    //else if(i==5){ value_f = 0.0/*1500 + ((float)(rci_msg.roll)/2047.0)*500*/; } // parameter 6
    //else if(i==6){ value_f = 1.2/*1500 + (((float)(rci_msg.thrust)/2047.0) - 1.0)*500*/; } // parameter 7
    //ROS_INFO("parameter: %d, value: %f",i,value_f);
    bytes = reinterpret_cast<uint8_t*>(&value_rc);
    mavmsg_ci.payload.push_back(bytes[0]);
    mavmsg_ci.payload.push_back(bytes[1]);
    //mavmsg_ci.payload.push_back(bytes[2]);
    //mavmsg_ci.payload.push_back(bytes[3]);
  }

  //mavmsg_ci.payload.push_back(176); // mav command do set mode -- 16 bit
  //t_pose += 1;
  //mavmsg_ci.payload.push_back(0);
  mavmsg_ci.payload.push_back(1); // system id
  mavmsg_ci.payload.push_back(250); //250 mp
  //mavmsg_ci.payload.push_back(tx_num); // transmission number

  // publish = true, heartbeat = false, control = true
  pubGeneral(true,false,true);/**/
 }
}

// callback for state
void mmaTranslation::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
  // obtain message
  current_state = (*msg);
  
  switch(current_state.state){
    case collab_msgs::SubjectCtrlState::ESTOP:
    case collab_msgs::SubjectCtrlState::OFF:
      if(time_start == time_now){ time_start = time_now = ros::Time::now().toSec(); }
      while(time_now - time_start < 1.0){ time_now = ros::Time::now().toSec(); } 
      tx_num = 0;
      /*mavmsg_sm.len = 6;
        mavmsg_sm.msgid = 66;
        pubGeneral(false,false,false);
        //payload bytes: 0, 0, 1, 1, 0, 1
      */ 
      //
      mavmsg_sm.len = 18;
      mavmsg_sm.msgid = 70;
      pubGeneral(false,false,false);

      // payload
      for(int i=0;i<7;i++){
        value_rc = 1500;
        if(i == 4){ value_rc = 960; }
        //else if(i == 5){ value_f = -96.705344; }
        bytes = reinterpret_cast<uint8_t*>(&value_rc);
        mavmsg_sm.payload.push_back(bytes[0]);
        mavmsg_sm.payload.push_back(bytes[1]);
        //mavmsg_sm.payload.push_back(bytes[2]);
        //mavmsg_sm.payload.push_back(bytes[3]);
      }
      //mavmsg_sm.payload.push_back(176); // mav command 17 mode changed, why? 
      //mavmsg_sm.payload.push_back(0);
      mavmsg_sm.payload.push_back(1); // system id
      mavmsg_sm.payload.push_back(255);
      //mavmsg_sm.payload.push_back(1); // transmission number

      // publish
      pubGeneral(true,false,false);/**/
      break; // data stream request message - 66
    case collab_msgs::SubjectCtrlState::STARTUP:
      if(count == 0){
        // mission item -- command 400. param1 = 1 to arm, 0 disarm
        mavmsg_sm.len = 33;
        mavmsg_sm.msgid = 76;
        pubGeneral(false,false,false);

        // payload
        value_f = 1.0; // first parameter
        bytes = reinterpret_cast<uint8_t*>(&value_f);
        for(int i=0;i<7;i++){
          if(i==1){ value_f = 0.0; bytes = reinterpret_cast<uint8_t*>(&value_f); }
          mavmsg_sm.payload.push_back(bytes[0]);
          mavmsg_sm.payload.push_back(bytes[1]);
          mavmsg_sm.payload.push_back(bytes[2]);
          mavmsg_sm.payload.push_back(bytes[3]); 
        }
        mavmsg_sm.payload.push_back(144); // mav command do set mode -- 16 bit
        mavmsg_sm.payload.push_back(1);
        mavmsg_sm.payload.push_back(1); // system id
        mavmsg_sm.payload.push_back(250); // 255 or 250
        mavmsg_sm.payload.push_back(tx_num); // transmission number

        // publish
        pubGeneral(true,false,false); tx_num++;
      }else if(count == 2000){ count = -1; }
      count++; 
      break;
    case collab_msgs::SubjectCtrlState::SHUTDOWN:
      if(count == 0){
        // mission item -- command 400. param1 = 1 to arm, 0 disarm
        mavmsg_sm.len = 33;
        mavmsg_sm.msgid = 76;
        pubGeneral(false,false,false);

        // payload
        value_f = 0.0; // first parameter
        bytes = reinterpret_cast<uint8_t*>(&value_f);
        for(int i=0;i<7;i++){
          if(i==1){ value_f = 0.0; bytes = reinterpret_cast<uint8_t*>(&value_f); }
          mavmsg_sm.payload.push_back(bytes[0]);
          mavmsg_sm.payload.push_back(bytes[1]);
          mavmsg_sm.payload.push_back(bytes[2]);
          mavmsg_sm.payload.push_back(bytes[3]); 
        }
        mavmsg_sm.payload.push_back(144); // mav command do set mode -- 16 bit
        mavmsg_sm.payload.push_back(1);
        mavmsg_sm.payload.push_back(1); // system id
        mavmsg_sm.payload.push_back(250);
        mavmsg_sm.payload.push_back(tx_num); // transmission number

        // publish
        pubGeneral(true,false,false); tx_num++;
      }else if(count == 2000){ count = -1; }
      count++;
      break;
    case collab_msgs::SubjectCtrlState::IDLE:
      // calibrate barometer -- wait 3 seconds
      if(time_start == time_now){ time_start = time_now = ros::Time::now().toSec(); }
      while(time_now - time_start < 20.0){ time_now = ros::Time::now().toSec(); }

      //
      mavmsg_sm.len = 18;
      mavmsg_sm.msgid = 70;
      pubGeneral(false,false,false);

      // payload
      for(int i=0;i<8;i++){ // rpty
        value_rc = 1500;
        if(i == 2){ value_rc = 1900; }
        //else if(i == 4){ value_f = 2.0; }
        //else if(i == 5){ value_f = -96.705344; }  
        bytes = reinterpret_cast<uint8_t*>(&value_rc);
        mavmsg_sm.payload.push_back(bytes[0]);
        mavmsg_sm.payload.push_back(bytes[1]);
      }
      mavmsg_sm.payload.push_back(1); // system id
      mavmsg_sm.payload.push_back(250);

      // publish
      pubGeneral(true,false,false);  /**/

      //
      /*mavmsg_sm.len = 33;
      mavmsg_sm.msgid = 76;
      pubGeneral(false,false,false);

      // payload
      for(int i=0;i<7;i++){
        value_f = 0.0;
        if(i == 0){ value_f = 0.5; }
        else if(i == 4){ value_f = 40.819533; }
        else if(i == 5){ value_f = -96.705344; }
        bytes = reinterpret_cast<uint8_t*>(&value_f);
        mavmsg_sm.payload.push_back(bytes[0]);
        mavmsg_sm.payload.push_back(bytes[1]);
        mavmsg_sm.payload.push_back(bytes[2]);
        mavmsg_sm.payload.push_back(bytes[3]);
      }
      mavmsg_sm.payload.push_back(16); // mav command 17 mode changed, why? 
      mavmsg_sm.payload.push_back(0);
      mavmsg_sm.payload.push_back(1); // system id
      mavmsg_sm.payload.push_back(250);
      mavmsg_sm.payload.push_back(1); // transmission number

      // publish
      pubGeneral(true,false,false);*/

      // publish ok to launch
      enable_launch.busy = false;
      enable_launch.header.stamp = ros::Time::now();
      not_busy.publish(enable_launch);
      break;
    case collab_msgs::SubjectCtrlState::LAUNCH:
      mavmsg_sm.len = 18;
      mavmsg_sm.msgid = 70;
      pubGeneral(false,false,false);

      // payload 
      mavmsg_sm.payload.clear();
      for(int i=0;i<8;i++){
        value_rc = 1500;
        if(i == 2){ value_rc = 1500; }
        //else if(i == 4){ value_f = 40.819533; }
        //else if(i == 5){ value_f = -96.705344; }
        //else if(i == 6){ value_f = 1.2; }
        bytes = reinterpret_cast<uint8_t*>(&value_rc);
        mavmsg_sm.payload.push_back(bytes[0]);
        mavmsg_sm.payload.push_back(bytes[1]);
        //mavmsg_sm.payload.push_back(bytes[2]);
        //mavmsg_sm.payload.push_back(bytes[3]);
      }

      //mavmsg_sm.payload.push_back(22); // mav command nav takeoff, fails? -- 16 bit
      //mavmsg_sm.payload.push_back(0);
      mavmsg_sm.payload.push_back(1); // system id
      //mavmsg_sm.payload.push_back(250);
      mavmsg_sm.payload.push_back(250); // transmission number

      // publish = true, heartbeat = false, control = true
      pubGeneral(true,false,false); 
      break;
    case collab_msgs::SubjectCtrlState::LAND: break;
    case collab_msgs::SubjectCtrlState::HOVER:
    case collab_msgs::SubjectCtrlState::TASK:
      mavmsg_sm.len = 18;
      mavmsg_sm.msgid = 70;
      pubGeneral(false,false,false);

      // payload 
      mavmsg_sm.payload.clear();
      for(int i=0;i<8;i++){
        value_rc = 1500;
        if(i == 0){ value_rc = 1800; }
        //else if(i == 4){ value_f = 40.819533; }
        //else if(i == 5){ value_f = -96.705344; }
        //else if(i == 6){ value_f = 1.2; }
        bytes = reinterpret_cast<uint8_t*>(&value_rc);
        mavmsg_sm.payload.push_back(bytes[0]);
        mavmsg_sm.payload.push_back(bytes[1]);
        //mavmsg_sm.payload.push_back(bytes[2]);
        //mavmsg_sm.payload.push_back(bytes[3]);
      }

      //mavmsg_sm.payload.push_back(22); // mav command do set mode -- 16 bit
      //mavmsg_sm.payload.push_back(0);
      mavmsg_sm.payload.push_back(1); // system id
      //mavmsg_sm.payload.push_back(250);
      mavmsg_sm.payload.push_back(250); // transmission number

      // publish = true, heartbeat = false, control = true
      pubGeneral(true,false,false);
      break;
  }
}

// heart beat timer callback
void mmaTranslation::heartBeat(const ros::TimerEvent& event){
  //ROS_INFO("in heart beat");// unique header data
  mavmsg_hb.len = 9;
  mavmsg_hb.msgid = 0;
  // same header data
  pubGeneral(false,true,false);

  // payload
  mavmsg_hb.payload.clear();
  //if(mode == 0){ mode = 89; }
  //else if(mode > 128){ mode = 217; custom_mode = 2; }
  mavmsg_hb.payload.push_back(custom_mode); // lsB of custom mode 
  mavmsg_hb.payload.push_back(0);
  mavmsg_hb.payload.push_back(0);
  mavmsg_hb.payload.push_back(0);
  mavmsg_hb.payload.push_back(2); // autopilot
  mavmsg_hb.payload.push_back(3); // type
  mavmsg_hb.payload.push_back(mode); // base mode 
  mavmsg_hb.payload.push_back(3); // system state
  mavmsg_hb.payload.push_back(3); // mavlink version
        
  // calculate crc and publish
  pubGeneral(true,true,false);
}

// general publishing 
void mmaTranslation::pubGeneral(bool publish, bool is_heart_beat,bool is_control){
  if(is_heart_beat){ mavmsg_tx = mavmsg_hb; }
  else if(is_control){ mavmsg_tx = mavmsg_ci; }
  else{ mavmsg_tx = mavmsg_sm; }
  if(publish){
    // fill value_u for cyclic redundancy check (crc) processing with payload data
    for(int i=0;i<(int)(mavmsg_tx.len);i++){ value_u.push_back(mavmsg_tx.payload[i]); }
    // add mavlink crc seed to value_u for crc processing 
    value_u.push_back(crc_seed[mavmsg_tx.msgid]);
    //ROS_INFO("crc seed: %u",crc_seed[mavmsg_tx.msgid]); 
    // calculate crc
    mavmsg_tx.crc = crc_calc();
    value_u.clear();
    // publish mavlink message
    mavmsg_tx.is_valid = true;
    mavmsg_tx.header.stamp = ros::Time::now();
    mavlink_transmit.publish(mavmsg_tx);
    seq = mavmsg_tx.seq; // update current sequence number
  }else{ 
    // update and add to vector for crc processing
    // message length, new sequence number, sysid, and compid
    value_u.push_back(mavmsg_tx.len);
    mavmsg_tx.seq = seq+1 % 256; value_u.push_back(mavmsg_tx.seq);
    mavmsg_tx.sysid = 1; value_u.push_back(mavmsg_tx.sysid);
    mavmsg_tx.compid = 1; value_u.push_back(mavmsg_tx.compid);
    value_u.push_back(mavmsg_tx.msgid);
    mavmsg_tx.payload.clear(); // reuse previous payload vector
    if(is_heart_beat){ mavmsg_hb = mavmsg_tx; }
    else if(is_control){ mavmsg_ci = mavmsg_tx; }
    else{ mavmsg_sm = mavmsg_tx; }
  }
}

// hash: crc16 x.25
uint16_t mmaTranslation::crc_calc(){
  crc = 0xffff;

  // invert bytes and possibly (~) bits then calculate crc
  for(int i=0;i<(int)(mavmsg_tx.len)+6;i++){ byte_crc_calc(value_u[i]); }
  
  return crc;
}

// per byte calculation of crc16 CCITT
void mmaTranslation::byte_crc_calc(uint8_t data){ 
  crc8 = (uint8_t)data ^ (uint8_t)(crc & 0x00ff);
  crc8 ^= (crc8<<4);
  crc = (crc>>8) ^ (crc8<<8) ^ (crc8<<3) ^ (crc8>>4);
}

/*// callback for robot control input
void mmaTelemetry::callbackControlInput(const boost::shared_ptr<collab_msgs::AsctecCtrlInput const> &msg){
    // create message copy
    rci_msg = (*msg);

    string start_str = ">*>254";

    // 
    message.data.insert(message.data.end(), start_str.begin(),start_str.end());
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.pitch), reinterpret_cast<uint8_t*>(&rci_msg.pitch)+sizeof(rci_msg.pitch));
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.roll), reinterpret_cast<uint8_t*>(&rci_msg.roll)+sizeof(rci_msg.roll));
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.yaw), reinterpret_cast<uint8_t*>(&rci_msg.yaw)+sizeof(rci_msg.yaw));
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.thrust), reinterpret_cast<uint8_t*>(&rci_msg.thrust)+sizeof(rci_msg.thrust));
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.ctrl), reinterpret_cast<uint8_t*>(&rci_msg.ctrl)+sizeof(rci_msg.ctrl));
    message.data.insert(message.data.end(),reinterpret_cast<uint8_t*>(&rci_msg.chksum), reinterpret_cast<uint8_t*>(&rci_msg.chksum)+sizeof(rci_msg.chksum));

    // publish tx buf msg
    mit_asctec_tx.publish(message);
}*/

/*// callback for polling asctec packet
void mmaTelemetry::callbackPoll(const boost::shared_ptr<collab_msgs::AsctecPoll const> &msg){
  // create poll message copy
  poll_msg = (*msg);

  string start_str = ">*>254";

  // fill in poll packet
  message.data.insert(message.data.end(), start_str.begin(), start_str.end());
  message.data.insert(message.data.end(), reinterpret_cast<uint8_t *>(&poll_msg.packets), reinterpret_cast<uint8_t *>(&poll_msg.packets)+sizeof(poll_msg.packets));

  // publish message
  mit_asctec_tx.publish(message);
}*/

