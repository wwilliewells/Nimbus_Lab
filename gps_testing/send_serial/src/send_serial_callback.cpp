/*
 * FILE: gps_device_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/gps_device.h" 

// callback for rf transmit <-- device serial port receive <-- gps transmit
void gpsDevice::callbackTransmit(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  if(!pause){
    // transmit base raw received data to other device or pose to rf
    // header and footer: device number 
    for(int j=0;j<3;j++){ tx_data.data.push_back(device); }
    for(int i=0;i<msg->data.size();i++){ tx_data.data.push_back(msg->data.at(i)); }
    for(int j=0;j<3;j++){ tx_data.data.push_back(device); }

    transmit_pub.publish(tx_data);
    tx_data.data.clear();
  }
}

// callback for rf receive --> device serial port transmit --> gps receive
void gpsDevice::callbackReceive(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  // transmit rf received data to device only if robot
  int i=-1; // iterator used outside of it's loop
  if(robot){
    if(device_data < 3){ // check for data addressed to this device
      for(i=0;i<msg->data.size();i++){ 
        if(msg->data.at(i) == device){ device_data++; } 
        if(device_data == 3){ break; }
      }
    }
    else{ // process data addressed to this device
      for(int j=i+1;j<msg->data.size();j++){
        // terminate device stream, if single device identifier in stream do not terminate 
        if(msg->data.at(j) == device){ device_data++; }
        else{ while(device_data > 3){ rx_data.data.push_back(device); device_data--; } }
        // device data stream ended, publish data to device
        if(device_data == 6){
          device_data = 0;
          receive_pub.publish(rx_data);
          rx_data.data.clear();
          break; 
        } // add byte to data stream
        else if(device_data == 3){ rx_data.data.push_back(msg->data.at(j)); }
      }
    }
  } 
}

// callback for test phase
void gpsDevice::callbackPause(const boost::shared_ptr<gps_testing::Move const> &msg){
  if(msg->enable_rf_test == 1){ pause = true; }else{ pause = false; }
}

