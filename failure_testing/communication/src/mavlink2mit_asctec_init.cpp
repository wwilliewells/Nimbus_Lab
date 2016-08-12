/*
 * FILE: mavlink2mit_asctec_init.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

// class include
#include "../include/mavlink2MIT_AscTec.h"

// initialize parameters
void mmaTranslation::initParams(){
  // initialize private variables
  tx_num = mode = custom_mode = previous_mode = seq = count = 0;
  time_start = time_now = 0.0;

  // initialize status node
  status.flight_mode = 16417;
  status.battery_voltage_1 = 11500.00;
  status.battery_voltage_2 = 0.0;
  status.status = status.chksum_error = status.up_time = 0;
  status.compass_enabled = 1;

  // initialize gps node
  // latitude = 4519175; // 40.8 // longitude = 6869837; //-96 2/3
  gps.latitude = 4519175; //
  gps.longitude = 6869837; //
  // speed, heading, horizontal_accuracy, vertical_accuracy, speed_accuracy
  gps.speed_x = gps.speed_y = gps.heading = gps.satellites = gps.status =
  gps.horizontal_accuracy = gps.vertical_accuracy = gps.speed_accuracy = 0;

  // initialize imu node, no equivalent mavlink data for asctec field
  imu.angvel_nick = imu.angvel_roll = imu.angvel_yaw =
  imu.acc_x_calib = imu.acc_y_calib = imu.acc_z_calib =
  imu.acc_angle_nick = imu.acc_angle_roll = imu.acc_absolute_value =
  imu.mag_heading = imu.speed_x = imu.speed_y = imu.speed_z =
  imu.height = imu.dheight = imu.dheight_reference = imu.height_reference = 0;
}

// initialize subscribers
void mmaTranslation::initSubscribers(){
  // subscribe to communication robot_comm, relay to mavros
  mavlink_receive = nh.subscribe("mavlink_rx",100,&mmaTranslation::callbackMavlinkRx,this);

  // subscribe to PID controller
  control_input = nh.subscribe<collab_msgs::AsctecCtrlInput>("robot_ctrl_input",10,&mmaTranslation::callbackControlInput,this);

  // subscribe to state from state-machine and issue takeoff and landing commands
  command_state = nh.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state",100,&mmaTranslation::callbackState,this);

  // may need to subscribe to a poll mavlink message
}

// initialize publishers
void mmaTranslation::initPublishers(){
  // communicate with state machine
  mit_asctec_imu = nh.advertise<collab_msgs::AsctecImuCalcData>("robot_imu",1,true);
  mit_asctec_status = nh.advertise<collab_msgs::AsctecLlStatus>("robot_status", 1, true);
  mit_asctec_gps = nh.advertise<collab_msgs::AsctecGpsData>("robot_gps", 1, true);
  mit_asctec_rc = nh.advertise<collab_msgs::AsctecRcData>("rc_data",1,true);
  // mit_asctec_rotors = nh.advertise<collab_msgs::AsctecRotors>("ctrl_out", 10, true);

  // publish asctec protocol --> mavlink message 
  mavlink_transmit = nh.advertise<flight_testing::Mavlink>("mavlink_tx",1,true);

  // enable signal
  not_busy = nh.advertise<flight_testing::Arducopter>("arducopter",1,true);
}

// initialize timers
void mmaTranslation::initTimers(){
  heart_beat = nh.createTimer(ros::Duration(1.0), &mmaTranslation::heartBeat,this);
}
