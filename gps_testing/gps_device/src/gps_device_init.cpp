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

  // set if radio connected  
  pnh.param("radio",radio,true);

  decode = 0;
}

// initialize subscribers -- transmit and receive are with respect to rf
void gpsDevice::initSubscribers(){
  // subscribe to device serial port receive <-- gps transmit
  if(!radio){
    // subscribe to intermittent device topic -- transmit path rover <-- base
    transmit_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/a/reach_rx_data",20,
      &gpsDevice::callbackTransmit,this);

    // subscrib to reach serial port receive -- receive path rover --> base
    receive_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/a/device_tx_data",20,
      &gpsDevice::callbackReceive,this);
  }
  else{ 
    transmit_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/r/reach_rx_data",20,
      &gpsDevice::callbackTransmit,this);

    // subscribe to rf serial port receive
    receive_sub = nh.subscribe<std_msgs::UInt8MultiArray>("/a/device_rx_data",20,
      &gpsDevice::callbackReceive,this);
  }
}

// initialize publishers
void gpsDevice::initPublishers(){
  // publish gps estimate
  if(!radio){ 
    receive_pub = nh.advertise<std_msgs::UInt8MultiArray>("/a/reach_tx_data",20,true);
  
    // publish to intermittent topic subscribed to by rf node
    transmit_pub = nh.advertise<std_msgs::UInt8MultiArray>("/a/device_rx_data",20,true); 
  }
  else{ 
    receive_pub = nh.advertise<std_msgs::UInt8MultiArray>("/r/reach_tx_data",20,true);
  
    // publish to intermittent topic subscribed to by rf node
    transmit_pub = nh.advertise<std_msgs::UInt8MultiArray>("/a/device_tx_data",20,true);
  }
}

