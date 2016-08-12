/**
 * FILE: mavlink2MIT_AscTec_thread.cpp
 * AUTHOR: William Willie Wells
 * DATE: February 2016
 */

// includes
#include <mavlink2MIT_AscTec.h>

// start parse packet type thread
void mmaTranslation::startThread(){
  parse_thread = boost::shared_ptr<boost::thread>
    (new boost::thread(boost::bind(&mmaTranslation::parsePktTypeThread,this)));
}

// parse Mavlink packet type (message identification) thread
void mmaTranslation::parsePktTypeThread(){
  ros::Rate ctrl_rate(30);
  while(ros::ok()){
    // decode mavlink packets into mit_asctec packets
    if(mavmsg_received){ 
      mel.lock();
      switch(mavmsg_rx.msgid){
        case(0): /*Heartbeat*/
          if(mavmsg_rx.len == 9 && mavmsg_rx.payload[8] == 3){
            mode = mavmsg_rx.payload[6];
            custom_mode = mavmsg_rx.payload[0];
            if(previous_mode != mode){ ROS_INFO("mode: (%u,%u), previous %u",mode,custom_mode,previous_mode); } 
            previous_mode = mode;
            mav_state = mavmsg_rx.payload[7]; // system state (IDLE, etc)
            if(mavmsg_rx.seq < 20){
              ROS_INFO("version: %u, system state: %u, custom mode: (%u,%u,%u,%u), base mode: %u, autopilot type: %u, type: %u",mavmsg_rx.payload[8],mav_state,mavmsg_rx.payload[0],mavmsg_rx.payload[1],mavmsg_rx.payload[2],mavmsg_rx.payload[3],mode,mavmsg_rx.payload[4],mavmsg_rx.payload[5]);    
            }      
            // change flight and motor status depending on mode
            if(mode > 128){ status.motors_on = 1; status.flying = 1; status.flight_mode = 16485; }
            else{ status.motors_on = 0; status.flying = 0; status.flight_mode = 16481; }

            // publish status message with heartbeat
            status.header.stamp = ros::Time::now();
            mit_asctec_status.publish(status);

            // publish gps message with received heartbeat
            gps.header.stamp = ros::Time::now();
            mit_asctec_gps.publish(gps);
          }
          break;
        case(1): /* sys_status */ //printBytes();
          //ROS_INFO("raw load: %f, voltage: %u, current: %i, battery: %i", (float)(mavmsg_rx.payload[12] | (mavmsg_rx.payload[13])<<8)/10,(mavmsg_rx.payload[14] | mavmsg_rx.payload[15]<<8),(int16_t)(mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8)/*100*/,(int8_t)mavmsg_rx.payload[30]);//,
          status.cpu_load = mavmsg_rx.payload[12] | mavmsg_rx.payload[13]<<8;
          break;
        case(2): /*sys uptime*/ break;
        case(24): /*GPS_raw_int*/ //printBytes();
          /*t_pose = 8; 
          value_i = (int)(mavmsg_rx.payload[t_pose] | mavmsg_rx.payload[t_pose+1]<<8 | mavmsg_rx.payload[t_pose+2]<<16 | mavmsg_rx.payload[t_pose+3]<<24);
          ROS_INFO(" lattitude: %i",value_i);
          t_pose += 4;
          value_i = (int)(mavmsg_rx.payload[t_pose] | mavmsg_rx.payload[t_pose+1]<<8 | mavmsg_rx.payload[t_pose+2]<<16 | mavmsg_rx.payload[t_pose+3]<<24);
          ROS_INFO("longitude: %i",value_i);
          t_pose += 4;
          value_i = (int)(mavmsg_rx.payload[t_pose] | mavmsg_rx.payload[t_pose+1]<<8 | mavmsg_rx.payload[t_pose+2]<<16 | mavmsg_rx.payload[t_pose+3]<<24);
          ROS_INFO("altitude: %i",value_i);*/
          //ROS_INFO("lattitude: %i, longitude: %i, altitude: %i, ground_speed: %u, satellites: %u",(int)(mavmsg_rx.payload[8] | mavmsg_rx.payload[9]<<8 | mavmsg_rx.payload[10]<<16 | mavmsg_rx.payload[11]<<24),(int)(mavmsg_rx.payload[12] | mavmsg_rx.payload[13]<<8 | mavmsg_rx.payload[14]<<16 | mavmsg_rx.payload[15]<<24),(int)(mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8 | mavmsg_rx.payload[18]<<16 | mavmsg_rx.payload[19]<<24),(mavmsg_rx.payload[24] | mavmsg_rx.payload[25]<<8), mavmsg_rx.payload[29]); 
          // height
          gps.height = (mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8
            | mavmsg_rx.payload[18]<<16 | mavmsg_rx.payload[19]<<24);
          break;
        case(27): /*raw imu*/ 
          /*for(int i=0;i<26;i=i+2){ 
            ROS_INFO("value: %i",(int16_t)(mavmsg_rx.payload[i] | mavmsg_rx.payload[i+1]<<8)); 
          } ROS_INFO(" ");*/
          imu.acc_x = (int16_t)(mavmsg_rx.payload[8] | mavmsg_rx.payload[9]<<8);
          imu.acc_y = (int16_t)(mavmsg_rx.payload[10] | mavmsg_rx.payload[11]<<8);
          imu.acc_z = (int16_t)(mavmsg_rx.payload[12] | mavmsg_rx.payload[13]<<8);
          //TODO: mag,gyro_x,y,z: mavlink 16 bit --> asctec 32-bit
          imu.angle_roll = (int16_t)(mavmsg_rx.payload[14] | mavmsg_rx.payload[15]<<8);
          imu.angle_nick = (int16_t)(mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8);
          imu.angle_yaw = (int16_t)(mavmsg_rx.payload[18] | mavmsg_rx.payload[19]<<8);
          imu.mag_x = (int16_t)(mavmsg_rx.payload[20] | mavmsg_rx.payload[21]<<8);
          imu.mag_y = (int16_t)(mavmsg_rx.payload[22] | mavmsg_rx.payload[23]<<8);
          imu.mag_z = (int16_t)(mavmsg_rx.payload[24] | mavmsg_rx.payload[25]<<8);
          // publish imu
          imu.header.stamp = ros::Time::now();
          mit_asctec_imu.publish(imu);
          break;
        case(29): /*scaled_pressure*/
          /*ROS_INFO("pressure: absolute: %f, differential: %f, temperature (C): %f",mavmsg_rx.payload[]/1013.25,mavmsg_rx.payload[1]/1013.25,(float)mavmsg_rx.payload_i[0]*0.01);*/
          break;
        case(30):// printBytes();/*attitude*/
          /*ROS_INFO("roll: %f, pitch: %f, yaw: %f, roll_vel: %f, pitch_vel: %f, yaw_vel: %f",mavmsg_rx.payload[0]*PI,mavmsg_rx.payload[1]*PI,mavmsg_rx.payload[2]*PI,mavmsg_rx.payload[3]*PI,mavmsg_rx.payload[4]*PI,mavmsg_rx.payload[5]*PI);*/
          break;
        case(33): //printBytes();/*(integer ned) global position*/ 
          /*ROS_INFO("lattitude: %i, longitude: %i, altitude: AMSL: %i, WSG84: %i; ground_speed: x: %i, y: %i, z: %i; heading: %f",mavmsg_rx.payload[0],mavmsg_rx.payload[1],mavmsg_rx.payload[2],mavmsg_rx.payload[3],mavmsg_rx.payload[4],mavmsg_rx.payload[5],mavmsg_rx.payload[6],mavmsg_rx.payload[13]/100.0);*/
          break;
        case(35): /*raw rc channels*/ 
          ROS_INFO("rc channels: 1: %u, 2: %u, 3: %u, 4: %u, 5: %u, 6: %u, 7: %u, 8: %u",(mavmsg_rx.payload[4] | mavmsg_rx.payload[5]<<8),(mavmsg_rx.payload[6] | mavmsg_rx.payload[7]<<8),(mavmsg_rx.payload[8] | mavmsg_rx.payload[9]<<8),(mavmsg_rx.payload[10] | mavmsg_rx.payload[11]<<8), (mavmsg_rx.payload[12] | mavmsg_rx.payload[13]<<8),(mavmsg_rx.payload[14] | mavmsg_rx.payload[15]<<8),(mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8),(mavmsg_rx.payload[18] | mavmsg_rx.payload[19]<<8));
          remote_control.channels_in[0] = (mavmsg_rx.payload[4] | mavmsg_rx.payload[5]<<8);
          remote_control.channels_in[1] = (mavmsg_rx.payload[6] | mavmsg_rx.payload[7]<<8);
          remote_control.channels_in[2] = (mavmsg_rx.payload[8] | mavmsg_rx.payload[9]<<8);
          remote_control.channels_in[3] = (mavmsg_rx.payload[10] | mavmsg_rx.payload[11]<<8);
          remote_control.channels_in[4] = (mavmsg_rx.payload[12] | mavmsg_rx.payload[13]<<8);
          remote_control.channels_in[5] = (mavmsg_rx.payload[14] | mavmsg_rx.payload[15]<<8);
          remote_control.channels_in[6] = (mavmsg_rx.payload[16] | mavmsg_rx.payload[17]<<8);
          remote_control.channels_in[7] = (mavmsg_rx.payload[18] | mavmsg_rx.payload[19]<<8);
          remote_control.header.stamp = ros::Time::now();
          mit_asctec_rc.publish(remote_control);
          break;
        case(36): /*rc channels scaled*/ //printBytes();/**/
          //ROS_INFO("rc channels: 1: %u, 2: %u, 3: %u, 4: %u, 5: %u, 6: %u, 7: %u, 8: %u",mavmsg_rx.payload[2],mavmsg_rx.payload[3],mavmsg_rx.payload[4],mavmsg_rx.payload[5],mavmsg_rx.payload[6],mavmsg_rx.payload[7],mavmsg_rx.payload[8],mavmsg_rx.payload[9]); 
          break;
        case(39):
          //ROS_INFO("sysid: %u, compid: %u, seq: %u, frame: %u, command: %u, current: %u, autocontinue: %u, param1: %u",mavmsg_rx.payload[32],mavmsg_rx.payload[33],(mavmsg_rx.payload[28] | mavmsg_rx.payload[29]<<8),mavmsg_rx.payload[34],(mavmsg_rx.payload[30] | mavmsg_rx.payload[31]<<8),mavmsg_rx.payload[35],mavmsg_rx.payload[36],(mavmsg_rx.payload[0] | mavmsg_rx.payload[1]<<8 | mavmsg_rx.payload[2]<<16 | mavmsg_rx.payload[3]<<24));
          break;
        case(42): /*mission_current*/ //printBytes();
          break;
        case(47): /*mission ack*/ 
          ROS_INFO("mission result: %u",mavmsg_rx.payload[2]);
          break;
        case(62): /*nav_controller_output*/ //target waypose (pitch, roll, and yaw) 
          /*printBytes(); // nav bearing [4], nav target [5]
          ROS_INFO("target: roll: %f, pitch: %f, heading: %f, bearing: %f; distance to target: %u",load_f[0]*360.0,load_f[1]*360.0,(float)load_i[0]/100.0,(float)load_i[1]/100.0,mavmsg.payload[6]);
      */
          break;
        case(74): /*vfr_hud*/ break;
        case(77): 
          ROS_INFO("cmd: %u, result: %u",mavmsg_rx.payload[0] | mavmsg_rx.payload[1]<<8,mavmsg_rx.payload[2]); 
          break;
        case(150): //printBytes();/*sensor offsets*/ 
          /*ROS_INFO("magnetometer: x: %d, y: %d, z: %d, r: %f; pressure %f, temperature: %f; gyroscope: x: %f, y: %f, z: %f; accelerometer: x: %f, y: %f, z: %f",load_i[0],load_i[1],load_i[2],load_f[0],(float)load_i[3]/1013.25,(float)load_i[4]*0.01,load_f[1],load_f[2],load_f[3],load_f[4],load_f[5],load_f[6]);*/
          break;
        case(151): /*set mag offsets; */ break;
        case(152): /*printBytes(); *mem info: heap*/ break;
        case(162): /*fence status*/ if(mavmsg_rx.payload[7] != 0){ printBytes(); } break;
        case(163): //printBytes();/*attitude and heading reference system*/ 
          /*ROS_INFO("gyro drift: x: %f, y: %f, z: %f; \naverage accel weight: %f; average renormalization: %f;\n average error: roll_pitch: %f, yaw: %f",load_f[0],load_f[1],load_f[2],load_f[3],load_f[4],load_f[5],load_f[6]);*/
          break;
        case(165): //printBytes();/*hardware status*/ 
          /*ROS_INFO("VCC: %f",(float)load_i[0]/1000.0);*/
          break;
        case(253):
          char error_msg[50];
          for(int i=0;i<mavmsg_rx.len;i++){ error_msg[i] = mavmsg_rx.payload[i]; }
          ROS_INFO("%s",error_msg);
          break; 
        default: 
          ROS_INFO("received message id: %i, message length: %u",mavmsg_rx.msgid,mavmsg_rx.len);
          if(mavmsg_rx.msgid == 254){  
            if(mavmsg_rx.payload.size() > 4){ 
              ROS_INFO("possible correct msgid: %u", mavmsg_rx.payload[4]); }
            else if(mavmsg_rx.payload.size() == 4){ 
              ROS_INFO("possible correct msgid: %u", (mavmsg_rx.crc & 0xff00)>>8);
            }else if(mavmsg_rx.payload.size() == 3){
              ROS_INFO("possible correct msgid: %u", mavmsg_rx.crc & 0x00ff);
            }
          }
          break;
      }
      mavmsg_received = false;    
      mel.unlock();
    }
    ctrl_rate.sleep();
  }
}

// print payload bytes
void mmaTranslation::printBytes(){
  for(int i=0;i<(int)(mavmsg_rx.len);i++){ 
    ROS_INFO("byte: %i value: %u",i,(mavmsg_rx.payload[i]));
  }
}

// stop parse rx data thread
void mmaTranslation::stopThread(void){ parse_thread->join(); }

