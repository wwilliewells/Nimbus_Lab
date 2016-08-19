/*
 * FILE: gps_device_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/gps_device.h" 

// callback for rover <-- base
void gpsDevice::callbackTransmit(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  //if(radio){ 
    // insert header into buffer 
    tx_data.data.push_back(device);
    tx_data.data.push_back(23);
    tx_data.data.push_back(device+1);
  //}

  // insert data into buffer
  for(int i=0;i<(int)msg->data.size();i++){ tx_data.data.push_back(msg->data.at(i)); }

  //if(radio){
    // insert footer into buffer
    tx_data.data.push_back(9);
    tx_data.data.push_back(12);
  //}

  // publish buffer
  transmit_pub.publish(tx_data);
  tx_data.data.clear();
}

// callback for rover --> base
void gpsDevice::callbackReceive(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  // publish to base station -- close loop
  for(int i=0;i<(int)msg->data.size();i++){
    switch(decode){
      case 0: // scan data stream for relevant data
        if(msg->data.at(i) == device){ decode = 1; } break;
      case 1: // potential relevant data found 
        if(msg->data.at(i) == 23){ decode = 2; } 
        else if(msg->data.at(i) == device){ decode = 1; } 
        else{ decode = 0; } 
        break;
      case 2: // relevant data found
        if(msg->data.at(i) == device+1){ decode = 3; } 
        else{ decode = 0; } 
        break;
      case 3: // process data stream
        if(msg->data.at(i) == 9){ decode = 4; } 
        else{ rx_data.data.push_back(msg->data.at(i)); } 
        break;
      case 4: // potential end of data stream
        if(msg->data.at(i) == 12){ 
          receive_pub.publish(rx_data); 
          rx_data.data.clear(); 
          decode = 0;
        }
        else if(msg->data.at(i) == 9){ rx_data.data.push_back(9); decode = 4; }
        else{ decode = 3; rx_data.data.push_back(9); rx_data.data.push_back(msg->data.at(i)); }
        break;
      default: ROS_WARN(" state %d should not exist",decode); break;
    }
  }
}

