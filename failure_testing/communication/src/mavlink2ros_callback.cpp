/*
 * FILE: mavlink2ros_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2ros.h" 

uint8_t msg_len[256] = 
  {9,31,12,0,14,28,3,32,0,0,0,6,0,0,0,0,0,0,0,0,20,2,25,23,30,101,22,26,16,14,28,32,
  28,28,22,22,21,6,6,37,4,4,2,2,4,2,2,3,13,12,37,0,0,0,27,25,0,0,0,0,0,68,26,185,
  229,42,6,4,0,11,18,0,0,37,20,35,33,3,0,0,0,22,39,37,53,51,53,51,0,28,56,42,33,0,0,0,
  0,0,0,0,26,32,32,20,32,62,44,64,84,9,254,16,12,36,44,64,22,6,14,12,97,2,2,113,35,6,79,35,
  35,22,13,255,14,18,43,8,22,14,36,43,41,32,243,14,0,0,100,36,60,30,42,8,4,12,15,13,6,15,14,0,
  12,3,8,28,44,3,9,22,12,18,34,66,98,8,48,39,3,20,24,29,44,4,40,2,206,7,29,0,0,0,0,27,
  44,22,25,0,0,0,0,0,42,14,0,0,0,0,0,0,0,0,0,0,0,0,8,3,3,6,7,2,0,0,0,0,
  0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,52,53,6,2,38,0,254,36,30,18,18,51,9,0};

// callback on serial data received
void mrTelemetry::callbackRobotRx(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  // obtain message and the size of the packet
  message_rx = (*msg);
  size = message_rx.data.size();   

  //ROS_INFO("in robot receive data callback");
  // process each byte in the message packet separately 
  for(unsigned int i=0;i<size;i++){ c = message_rx.data.at(i); processByte(); }
}

// process individual bytes and publish mavlink_receive values
void mrTelemetry::processByte(){
  // state specific actions: change state, publish complete message
  switch(state){
    case START:
      if(c == 254){ state = PAYLOAD_LEN; }
      //else{ ROS_INFO("Non-start byte in start state: %u",c); } 
      break;
    case PAYLOAD_LEN:
      if(c == 0){ state = START; /*ROS_INFO("len error, length: %u",c);*/ }
      else{ state = SEQUENCE; mavmsg_rx.len = c; mavmsg_rx.payload.clear(); }
      break;
    case SEQUENCE:
      //if(c - seq != 1){ 
        //ROS_INFO("previous seq: %d, current seq: %d, missed packets: %d",seq,c,c-seq-1); }  
      state = SYSTEM_ID; mavmsg_rx.seq = c;
      break;
    case SYSTEM_ID:
      if(c != 1){                     
        //ROS_INFO("sysid error, len: %u, seq: %u, sysid: %u",mavmsg_rx.len,mavmsg_rx.seq,c);
        state = START;
      }else{ state = COMPONENT_ID; mavmsg_rx.sysid = c; }
      break;
    case COMPONENT_ID:
      if(c != 1){ 
        //ROS_INFO("compid error, len: %u, seq: %u, sysid: %u, compid: %u",mavmsg_rx.len,mavmsg_rx.seq,mavmsg_rx.sysid,c); 
        state = START; }
      else{ state = MESSAGE_ID; mavmsg_rx.compid = c; }
      break;
    case MESSAGE_ID:
      if(c != 253 && msg_len[c] != mavmsg_rx.len){
        //ROS_INFO("msgid error, len: %u, seq: %u, sysid: %u, compid: %u, msgid: %u",mavmsg_rx.len,mavmsg_rx.seq,mavmsg_rx.sysid,mavmsg_rx.compid,c);
        state = START;
      }else{ state = PAYLOAD; mavmsg_rx.msgid = c; tmp = 1; }
      break;
    case PAYLOAD:
      // push payload value until end of payload
      if(tmp <= mavmsg_rx.len){ 
        mavmsg_rx.payload.push_back(c);
        if(tmp == mavmsg_rx.len){ switch_state = true; }
      }
      tmp++;

      if(switch_state){ state = CRC; switch_state = false; crc = 0; tmp = 2; }
      break;
    case CRC:
      //ROS_INFO("CRC len: %i, byte: %i",bytes,tmp);
      crc |= c << (bytes - tmp)*8; 
      if(tmp == 1){
        // set cyclic redundancy check
        mavmsg_rx.is_valid = true;
        mavmsg_rx.crc = crc;

        // publish entire message
        mavmsg_rx.header.stamp = ros::Time::now();
        mavlink_receive.publish(mavmsg_rx); 
        switch_state = true;
      }
      tmp--;

      if(switch_state){ state = START; switch_state = false; seq = mavmsg_rx.seq; }
      break;
      // validate received crcs? crc16 x25
    default: state = START; break;
  }
}

// callback for mavlink transmission
void mrTelemetry::callbackMavlinkTx(const boost::shared_ptr<flight_testing::Mavlink const> &msg){
  //ROS_INFO("in transmit callback");
  mavmsg_tx = (*msg);

  // correct layout dimension data
  message_tx.layout.dim[0].label = "";
  message_tx.layout.dim[0].stride = message_tx.layout.dim[0].size = (uint32_t)(mavmsg_tx.len + 8);

  // convert ros topic into serial buffer using push_back
  message_tx.data.clear();
  message_tx.data.push_back(254);
  message_tx.data.push_back(mavmsg_tx.len);
  message_tx.data.push_back(mavmsg_tx.seq);
  message_tx.data.push_back(mavmsg_tx.sysid);
  message_tx.data.push_back(mavmsg_tx.compid);
  message_tx.data.push_back(mavmsg_tx.msgid);
  for(int i=0;i<mavmsg_tx.len;i++){ message_tx.data.push_back(mavmsg_tx.payload[i]); }
  message_tx.data.push_back((uint8_t)(mavmsg_tx.crc & 0x00FF));
  message_tx.data.push_back((uint8_t)((mavmsg_tx.crc & 0xFF00) >> 8));

  // publish tx buf msg
  robot_transmit.publish(message_tx); //ROS_INFO("message sent");
}

