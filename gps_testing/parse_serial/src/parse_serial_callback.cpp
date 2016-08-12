/*
 * FILE: parse_serial_callback.cpp
 * AUTHOR: William Willie Wells, modified from code
 *         written by Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2016
 */

// class include
#include "../include/parse_serial.h" 

// callback on serial data received
void ParseSerial::callbackSerialRx(const boost::shared_ptr<std_msgs::UInt8MultiArray const> &msg){
  mel.lock();
   
  // obtain message and the size of the packet
  message = (*msg);
  size = message.data.size();

  // process each byte in the message packet separately 
  for(unsigned int i=0;i<size;i++){
    c = message.data.at(i);
    processByte();
  }

  mel.unlock();
}

// process individual bytes and publish sensor values
void ParseSerial::processByte(){
  // read and construct uint16_t sensor values a byte at a time
  if(state != NOT_SYN && state != READY){ // construct complete sensor value
    if(c < 58 && c > 47){ if(value != 0){ value *= 10; } value += c - 48; }
  }

  // state specific actions: change state, publish sensor value
  switch(state){ 
    case NOT_SYN: /* '\r' */
      if(c == 13){ state = READY; }else{ state = NOT_SYN; }
      break;
    case READY: /* '\n' */
      if(c == 10){ state = SENSOR1; }else{ state = NOT_SYN; } value = 0;
      break;
    case SENSOR1:
      if(c == 44){ 
        sensor_value.value = value; 
        sensor_value.percent = ((float)value)/1024;
        sensor_value.distance = sensor_value.percent*9.8+0.2;
      } /* ',' */
      else if(c == 32){
        if(value < 1024){
          sensor_value.header.stamp = ros::Time::now(); 
          sensor.publish(sensor_value);
        }
        value = 0;
      } /* ' ' */
      else if(c == 13){ state = READY; }
      break;
    default: state = NOT_SYN; break;
  }
}

/*// callback for operating range
void ParseSerial::callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &msg){
    mel.lock();
    // indicate whether the vehicle is in state 7 or 8
    current_state = (*msg);
    if(current_state.state > 6 && flying == false){
        flying = true;
    }else if(current_state.state < 7 && flying == true){
        flying = false;
    }
    sensor_value.flying = flying;
    // state_received = true;
    mel.unlock();
}

// callback for task segmentation
void ParseSerial::callbackTask(const boost::shared_ptr<collab_msgs::SubjectPose const> &msg){
    mel.lock();
    // indicate which task sequence corresponds to a particular sensor value
    current_task = (*msg);
    sensor_value.task = current_task.header.seq;
    
    //
    mel.unlock();
}*/
