/*
 * FILE: acknowledge_control_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 */

// class include
#include "../include/acknowledge_control.h" 

// callback for robot receive
void Acknowledge::callbackData(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  //ROS_INFO("AT: %d, request: %d, count: %d, bytes: %d",at_enable,at_request,count,bytes);
  // parse acknowledgement data 
  int k,i=-1; // iterator used outside of it's loop
  if(!ack_data){ // check for data addressed to acknowledgement
    if(!at_request && !at_enable){
      for(i=0;i<msg->data.size();i++){ 
        if(msg->data.at(i) == 23){
          bytes++; 
          if(count < 2){ count++; }
          else{ ack_data = true; count = 0; break; } 
        }
      }
    }
    else if(!at_enable && at_request){ // requesting RSSI
      for(k=0;k<msg->data.size();k++){ 
        if(msg->data.at(k) == 13){
          // publish request for RSSI
          rdata.data.push_back(65);
          rdata.data.push_back(84);
          rdata.data.push_back(68);
          rdata.data.push_back(66);
          rdata.data.push_back(13);
          data_pub.publish(rdata);
          rdata.data.clear();
          at_enable = true; at_request = false;
        }
      }
    }
    else if(at_enable && !at_request){ // receiving RSSI
      for(k=0;k<msg->data.size();k++){ count++; if(count == 2){ break; } }
      if(count == 2){
        if(msg->data.at(k) < 101 && msg->data.at(k) > 35){
          // publish acknowledgement data
          acknowledge.signal_strength = -1*msg->data.at(k);
          acknowledge.bytes = bytes;
          acknowledge.source = 0; // seq should be same
          acknowledge.header.stamp = ros::Time::now();
          ack_pub.publish(acknowledge);
        }
        else{ 
          ROS_INFO("byte: %d,k: %d",msg->data.at(k),k);
        }
        bytes = count = 0;

        // exit AT mode
        rdata.data.push_back(65);
        rdata.data.push_back(84);
        rdata.data.push_back(67);
        rdata.data.push_back(78);
        rdata.data.push_back(13);
        data_pub.publish(rdata);
        rdata.data.clear();
        at_enable = rf_test = false;

        // publish end of rf_test
        rssi.enable_rf_test = rssi.end_test = 0;
        rssi.enable_gps_test = 1;
        rssi.header.stamp = ros::Time::now();
        test_pub.publish(rssi);
      }
    }
  }
  if(ack_data){ // process data addressed as acknowledgement
    for(int j=i+1;j<msg->data.size();j++){
      // add byte to ack topic
      if(count == 0){
        acknowledge.seq = msg->data.at(j);
      }
      else if(count == 1){ 
        acknowledge.bytes = msg->data.at(j)<<8; 
      }
      else if(count == 2){
        acknowledge.bytes |= msg->data.at(j);
      }
      else{ // process raw byte into RSSI value
        if(msg->data.at(j) < 101 && msg->data.at(j) > 35){ 
          acknowledge.signal_strength = -1*msg->data.at(j); 

          // publish ack data
          acknowledge.source = 1;
          acknowledge.header.stamp = ros::Time::now();
          ack_pub.publish(acknowledge);
        }
        else{ acknowledge.signal_strength = 0; }
        bytes = bytes + count;

        // request AT mode
        ros::Time begin = ros::Time::now();
        ros::Time now_time = ros::Time::now();

        // wait guard time
        while((now_time-begin).toSec() < 0.5){ now_time = ros::Time::now(); }

        // publish AT command mode
        for(int i=0;i<3;i++){ rdata.data.push_back(43);  }
        data_pub.publish(rdata);
        rdata.data.clear();
        at_request = true;

        // reset variables
        ack_data = false;
        stuck_count = count = 0;
        break;
      }
      count++;
    }
  }
}

// callback for test phase
void Acknowledge::callbackRSSI(const boost::shared_ptr<gps_testing::Move const> &msg){
  if(msg->enable_rf_test == 1){ 
    rf_test = true;
    rdata.layout.dim[0].label = "0";
    rdata.layout.dim[0].stride = rdata.layout.dim[0].size = 23 - 5;
    for(int i=0;i<22;i++){ rdata.data.push_back(i); }
    rdata.data.push_back(seq++);
    rdata.data.push_back(23);
    rdata.data.push_back(22);
    data_pub.publish(rdata);
    rdata.data.clear();
  }
  else{ rf_test = false; }
  full = msg->full_test;
}

// timer to publish data
void Acknowledge::rfData(const ros::TimerEvent&){
  if(first){
    // publish ack data 
    acknowledge.bytes = 0;
    acknowledge.signal_strength = -50;
    acknowledge.header.stamp = ros::Time::now();
    ack_pub.publish(acknowledge);
    first = false;
  }

  //rf_test = true;
  rdata.layout.dim[0].label = "0";
  rdata.layout.dim[0].stride = rdata.layout.dim[0].size = 23 - 5;
  for(int i=0;i<22;i++){ rdata.data.push_back(i); }
  rdata.data.push_back(seq++);
  rdata.data.push_back(23);
  rdata.data.push_back(22);
  data_pub.publish(rdata);
  rdata.data.clear();
  /*if(!rf_test){
    rssi.enable_gps_test = rssi.end_test = 0;
    rssi.enable_rf_test = 1;
    rssi.header.stamp = ros::Time::now();
    test_pub.publish(rssi);
  }
  else{ // stuck
    ROS_INFO("test state: %d",rf_test); 
    //if(stuck_count < 1){ stuck_count++; }
    //if(!full && stuck_count == 1){
      rssi.enable_gps_test = rssi.end_test = rssi.enable_rf_test = 0;
      rssi.header.stamp = ros::Time::now();
      test_pub.publish(rssi);
    //}
  }*/
}
