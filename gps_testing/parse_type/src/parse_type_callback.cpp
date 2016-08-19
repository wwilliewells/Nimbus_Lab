/*
 * FILE: parse_type_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/parse_type.h"

// parse and publish based on type 
void parse_type::callbackType(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  // 16-bit address packet
  if(msg->data.at(0) == 0x81){
    if((int)msg->data.size() < 4){ ROS_ERROR("XBee parsePacket(): no data in packet"); return; }
    if(msg->data.at(6) == 23 && msg->data.at(7) < 5){ // gps
      for(int i=5;i<(int)msg->data.size();i++){ device_data.data.push_back(msg->data.at(i)); }
      device_pub.publish(device_data);
      device_data.data.clear();  
    }
    else{ // rssi
      // publish control rssi
      rssi_data.source = msg->data.at(2);
      rssi_data.rssi = (uint8_t)msg->data.at(3);
      rssi_data.bytes = (int)msg->data.size() - 5;

      rssi_data.seq = msg->data.at(5);
      rssi_data.header.stamp = ros::Time::now();
      rssi_pub.publish(rssi_data);

      // publish robot rssi
      rssi_data.source = msg->data.at(7);
      rssi_data.rssi = (uint8_t)msg->data.at(8);
      rssi_data.bytes = msg->data.at(6);

      rssi_data.header.stamp = ros::Time::now();
      rssi_pub.publish(rssi_data);
    }
  } // transmit status message
  else if(msg->data.at(0) == 0x89){ 
    if(msg->data.at(2) != 0){ ROS_WARN("transmit status: %d",msg->data.at(2)); } 
  }
  else{ ROS_WARN("XBee parsePacket(): Unknown message type %#x",msg->data.at(0)); }
}

