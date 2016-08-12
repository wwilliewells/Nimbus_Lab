/*
 * FILE: parse_serial_callback.cpp
 * AUTHOR: William Willie Wells, modified from code
 *         written by Carrick Detweiler and Najeeb Najeeb
 * DATE: May 2015
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
  // read and construct floating point sensor values a byte at a time
  if(state != NOT_SYN && state != READY){
    if(c == 44 || (state == SENSOR4 && c == 13)){ // sensor value end condition
      // scale value based on control board used
      if(state == SENSOR1 || state == SENSOR3){ value /= 49.44; }
      else{ value /= 14.9; }
    } // reset flags
    else if(c == 32 || (state == SENSOR4 && c == 13)){ decimal = false; place = 0; }
    else if(c == 46){ decimal = true; } // decimal point, '.'  
    else if(c < 58 && c > 47){ // construct complete sensor value
      if(!decimal){ if(value != 0){ value *= 10; } value += c - 48; }
      else{ place++; value += (c -48)/(10*place); }
    }
  }

  // state specific actions: change state, publish sensor value
  switch(state){ 
    case NOT_SYN:
      if(c == 13){ state = READY; } // '\r'
        else{ state = NOT_SYN; }
        break;
    case READY:
      if(c == 10){ state = SENSOR1; } // '\n'
      else{ state = NOT_SYN; } value = 0.0;
      break;
    case SENSOR1:
      if(c == 44){ 
        if(value <= 51.8){ sensor_value.voltage_one = value; }
        value = 0.0;
      }
      else if(c == 32){ state = SENSOR2; value = 0.0; } // ' '
      break;
    case SENSOR2:
      if(c == 44){ 
        if(value <= 44.7){ sensor_value.current_one = value; }
        value = 0.0; 
      }
      else if(c == 32){ state = SENSOR3; value = 0.0; }
      break;
    case SENSOR3:
      if(c == 44){ 
        if(value <= 51.8){ sensor_value.voltage_two = value; }
        value = 0.0;
      }
      else if(c == 32){ state = SENSOR4; value = 0.0; }
      break;
    case SENSOR4:
      if(c == 13){
        if(value <= 44.7){ sensor_value.current_two = value; }
        value = 0.0;
        sensor_value.header.stamp = ros::Time::now(); 
        sensor.publish(sensor_value); 
        state = READY;
      }
      break;
    default: state = NOT_SYN; break;
  }
}

// callback for operating range
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
}
