/*
 * FILE: send_serial_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: July 2016
 */

// class include
#include "../include/send_serial.h"

// initialize parameters
void sendSerial::initTimers(){
  timer = nh.createTimer(ros::Duration(30.0),&sendSerial::send,this);
}

// publish auto start command
void sendSerial::send(const ros::TimerEvent& e){
  /*stream.data.push_back('A');
  stream.data.push_back('S');
  data_pub.publish(stream);
  stream.data.clear();
  stream.data.push_back('D');
  stream.data.push_back('T');
  data_pub.publish(stream);
  stream.data.clear();*/
  stream.data.push_back('L');
  stream.data.push_back('F');
  data_pub.publish(stream);
  stream.data.clear();
}

// initialize publishers
void sendSerial::initPublishers(){
  data_pub = nh.advertise<std_msgs::UInt8MultiArray>("laser_tx_data",1,true);
}

