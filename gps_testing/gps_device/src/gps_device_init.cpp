/*
 * FILE: gps_device_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: June 2016
 */

// class include
#include "../include/gps_device.h"

// initialize parameters
void gpsDevice::initParams(){
  // set gps device  
  pnh.param("device",device,1); 
  if(device < 1 || device > 3){ device = 1; }

  decode = 0;
}

// initialize subscribers -- transmit and receive are with respect to rf
void gpsDevice::initSubscribers(){
  // subscribe to device serial port receive <-- gps transmit
  if(device == 1){
    transmit_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/n/nshp_rx_data",1000,
      &gpsDevice::callbackTransmit,this);
  }
  else if(device == 2){ 
    transmit_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/r/reach_rx_data",1000,
      &gpsDevice::callbackTransmit,this);
  }
  else if(device == 3){ 
    transmit_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/p/piksi_rx_data",1000,
      &gpsDevice::callbackTransmit,this);
  }

  // subscribe to rf serial port receive
  device_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/a/device_rx_data",1000,
    &gpsDevice::callbackReceive,this);
}

// initialize publishers
void gpsDevice::initPublishers(){
  // publish gps estimate
  if(device == 1){ 
    receive_pub = nh.advertise<std_msgs::UInt8MultiArray>("/n/nshp_tx_data",1000,true); 
  }
  else if(device == 2){ 
    receive_pub = nh.advertise<std_msgs::UInt8MultiArray>("/r/reach_tx_data",1000,true);
  }
  else if(device == 3){ 
    receive_pub = nh.advertise<std_msgs::UInt8MultiArray>("/p/piksi_tx_data",1000,true);
  }
  
  // publish to intermittent topic subscribed to by rf node
  device_pub = nh.advertise<std_msgs::UInt8MultiArray>("/a/device_tx_data",1000,true);
}

