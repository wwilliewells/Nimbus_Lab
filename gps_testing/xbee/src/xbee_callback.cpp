/*
 * FILE: xbee_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/xbee.h"

// transmit device data in API mode
void XBee::callbackDevice(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  int checksum = 0;

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

  mel.lock();
  tx_pub.publish(deviceTx);
  mel.unlock();
  deviceTx.data.clear();
}

// create a timed data stream
void XBee::stream(const ros::TimerEvent& e){
  int checksum = 0;

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

  mel.lock();
  tx_pub.publish(streamData);
  mel.unlock();
  streamData.data.clear();
}

/**
 * Parse any data that has come in on the serial line
 **/
void XBee::callbackData(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  static uint8_t state = STATE_START;
  static uint16_t checksum = 0;
  static uint8_t flag = 0;
  static int total = 0;
  static int fail = 0;
  //static int percent = 0;

  // process byte stream
  //ROS_INFO("state: %u",state);//ROS_DEBUG_NAMED("RawRX","XBee rx callback:");
  for(int i=0;i<(int)msg->data.size();i++){
    c = msg->data.at(i);
    //ROS_INFO("state: %u, byte: %d, data: %u",state,i,c);//ROS_DEBUG_NAMED("RawRX"," %c",c);
    switch(state){
      case STATE_START:
        if(c == 0x7E){ state = STATE_LEN0; } break;
      case STATE_LEN0:
        if(c > 0){
          ROS_WARN("XBee callback(): at: %d max data len exceeded, got %d, max %d",
            i,c,MAX_DATA_LEN);
          if(c < MAX_DATA_LEN){ 
            length = c; 
            checksum = bytesRead = 0;
            dataRx.data.clear();
            state = STATE_DATA;
          }
          else{ state = STATE_START; }
          break;
        }
        length = c;
        state = STATE_LEN1;
        break;
      case STATE_LEN1:
        length = c + (length<<8);
        // Make sure we can fit this much data
        if(length > MAX_DATA_LEN || length == 0){
          ROS_WARN("XBee callback(): at %d len limits exceeded, got %d, max %d",i,
            length,MAX_DATA_LEN);
          state = STATE_START;
        }
        else{ checksum = bytesRead = 0; dataRx.data.clear(); state = STATE_DATA; }
        break;
      case STATE_DATA:
        checksum += c;
        checksum &= 0xff;
        if(bytesRead == 0){ if(c == 129){ flag = 1; }else{ flag = 0; } } 
        if(flag){ dataRx.data.push_back(c); }
        bytesRead++;
        // check if we are done
        if(bytesRead >= length){ total++; state = STATE_CHECKSUM; }
        break;
      case STATE_CHECKSUM:
        // Final computation
        checksum = 0xff - checksum;
        if(c != checksum){
          fail++;
          ROS_WARN("XBee callback(): Invalid checksum, at: %d, length: %d, fail: %f",
            i,length,100*(double)fail/((double)total));
          if(c == 0x7E && i < msg->data.size() - 1 && msg->data.at(i+1) == 0){
            state = STATE_LEN0; 
            break; 
          }
        }
        else{ rx_pub.publish(dataRx); }
        state = STATE_START;
        break;
      default: ROS_WARN(" state %u should not exist",state); state = STATE_START; break;
    }
  }
}

