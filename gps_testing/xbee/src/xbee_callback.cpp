/*
 * FILE: xbee_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/xbee.h"

// transmit device data in API mode
void XBee::callbackDevice(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  mel.lock();
  int checksum = 0;
  int j;
  // API send
  // start byte
  deviceTx.data.push_back(0x7E);
  // Send the correct length
  deviceTx.data.push_back((0)>>8);
  deviceTx.data.push_back((msg->data.size()+5)&0xff);
  // Send rest of header(-) and data(+) 
  for(int i=-5;i<(int)msg->data.size();i++){
    if(i==-5){ j = 0x01; }
    else if(i == -4){ j = d_seq++; }
    else if(i == -3){ j = 0x00; }
    else if(i == -2){ j = 0x00ff & rcvr; }
    else if(i == -1){ j = 0x00; }
    else{ j = msg->data.at(i); }
    deviceTx.data.push_back(j);
    checksum += j;
    checksum &= 0xff;
  }
  checksum = 0xff - checksum;
  deviceTx.data.push_back(checksum);

  data_pub.publish(deviceTx);
  deviceTx.data.clear();
  mel.unlock();
}

// create a timed data stream
void XBee::stream(const ros::TimerEvent& e){
  mel.lock();
  int checksum = 0;
  int j;
  // API send
  // Start byte
  streamData.data.push_back(0x7E);
  // Send the correct length
  streamData.data.push_back((0)>>8);
  streamData.data.push_back((0x1D)&0xff);
  // 
  for(int i=-5;i<24;i++){
    if(i==-5){ j = 0x01; }
    else if(i == -4){ j = seq++; }
    else if(i == -3){ j = 0x00; }
    else if(i == -2){ j = 0x00ff & rcvr; }
    else if(i == -1){ j = 0x00; } 
    else{ j = i; } 
    streamData.data.push_back(j); 
    checksum += j;
    checksum &= 0xff; 
  }
  checksum = 0xff - checksum;
  streamData.data.push_back(checksum);

  data_pub.publish(streamData);
  streamData.data.clear();
  mel.unlock();
}

// publish raw gps position estimates
void XBee::publishPose(unsigned char *data, int length){
  for(int i=5;i<length;i++){ deviceRx.data.push_back(data[i]); }
  device_pub.publish(deviceRx); // shared topic
  deviceRx.data.clear();
}

// publish RSSI
void XBee::publishRSSI(unsigned char *data, int length){
  // publish control rssi
  messageRx.source = data[2]; 
  messageRx.rssi = data[3];
  messageRx.bytes = length - 5;
  
  if(data[2] == rcvr & 0xff){ messageRx.seq = data[5]; }
  messageRx.header.stamp = ros::Time::now();
  rssi_pub.publish(messageRx);
 
  // publish robot rssi
  messageRx.source = data[7];
  messageRx.rssi = data[8];
  messageRx.bytes = data[6];

  messageRx.header.stamp = ros::Time::now();
  rssi_pub.publish(messageRx);
}

/**
 * Parse a valid API packet received from the zigbee.  This is all of
 * the data that is received (without the API packet headers and
 * checksum, but with the packet headers itself).
 **/
void XBee::parsePacket(unsigned char *data, int length){
  // 16-bit address packet
  if(data[0] == 0x81){
    //ROS_INFO(" receive bytes/or prot: %d",data[6]);
    if(length < 4){ ROS_ERROR("XBee parsePacket(): no data in packet"); return; }
    if(data[6] == 23){ publishPose(data,length); }
    else{ publishRSSI(data,length); }
  } // transmit status message
  else if(data[0] == 0x89){ if(data[2] != 0){ ROS_WARN("transmit status: %d",data[2]); } }
  else{ ROS_WARN("XBee parsePacket(): Unknown message type %#x",data[0]); }
}

/**
 * Parse any data that has come in on the serial line
 **/
void XBee::callbackData(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  static int state = STATE_START;
  static int length;
  static int bytesRead;
  static int checksum;
  static unsigned char data[MAX_DATA_LEN];

  unsigned char c;
  // process byte stream
  //ROS_INFO("state: %u",state);//ROS_DEBUG_NAMED("RawRX","XBee rx callback:");
  for(int i=0;i<msg->data.size();i++){
    total_bytes++;
    bytes_received++;
    c = msg->data.at(i);
    ROS_INFO("state: %u, byte: %d, data: %u",state,i,c);//ROS_DEBUG_NAMED("RawRX"," %c",c);
    switch(state){
      case STATE_START:
        if(c == 0x7E){ 
          byteLog.header.stamp =  ros::Time::now();
          state = STATE_LEN0; 
          length = 0;
        } 
        break;
      case STATE_LEN0:
        byteLog.total = total_bytes;
        byteLog.packet_size = bytes_received - 2;
        byte_pub.publish(byteLog);
        bytes_received = 2;
        if(c > 0){
          ROS_WARN("XBee callback(): at: %d max data len exceeded, got %d, max %d",i,c,MAX_DATA_LEN);
          state = STATE_START; return;
        }
        length = c;
        state = STATE_LEN1;
        break;
      case STATE_LEN1:
        len = length = c + (length<<8);
        // Make sure we can fit this much data
        if(length > MAX_DATA_LEN || length == 0){
          ROS_WARN("XBee callback(): at %d len limits exceeded, got %d, max %d",i,
            length,MAX_DATA_LEN);
          state = STATE_START; return;
        }
        else{ checksum = bytesRead = 0; state = STATE_DATA; }
        break;
      case STATE_DATA:
        if(c == 0x7E){ // data bytes dropped
          if(length - bytesRead == 1){ 
            byteLog.header.stamp =  ros::Time::now();
            state = STATE_LEN0; 
            break; 
          } 
        }
        checksum += (unsigned char) c;
        checksum &= 0xff;
        data[bytesRead++] = c;
        // check if we are done
        if(bytesRead >= length){ state = STATE_CHECKSUM; }
        break;
      case STATE_CHECKSUM:
        // Final computation
        checksum = 0xff - checksum;
        if(((unsigned char)c) != checksum){
          if(c == 0x7E){ // checksum missing
            byteLog.header.stamp =  ros::Time::now();
            state = STATE_LEN0; 
            break; 
          }
          ROS_WARN("XBee callback(): Invalid checksum, at: %d, length: %d, checksum: %d, calc cs: %d",
            i,length,c,checksum);
        }
        else{ parsePacket(data,length); checksum = 0; }
        state = STATE_START;
        return;
      default: state = STATE_START; break;
    }
  }
}

