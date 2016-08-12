/*
 * FILE: reach_callback.cpp
 * AUTHOR: William Willie Wells
 * DATE: August 2016 ...
 */

// class include
#include "../include/reach.h"

// decode Reach data
void reach::callbackReach(const boost::shared_ptr<std_msgs::UInt8MultiArray const>& msg){
  // publish gps data
  for(int i=0;i<(int)msg->data.size();i++){
    // publish gps data on a topic
    ROS_INFO(" stat: (%u,%u)",decode_nmea,decode_llh);
    switch(decode_nmea){
      case 0: // scan NMEA data for UBLOX data 
        if(msg->data.at(i) == 181){ decode_nmea = 1; } break;
      case 1: // potential UBLOX data found
        if(msg->data.at(i) == 98){ decode_nmea = 2; }
        else if(msg->data.at(i) == 181){ decode_nmea = 1; }
        else{ decode_nmea = 0; }
        break;
      case 2: // check for nav data
        if(msg->data.at(i) == 1){ decode_nmea = 3; }else{ decode_nmea = 0; } break;
      case 3: // check for llh data
        if(msg->data.at(i) == 2){ decode_nmea = 4; }else{ decode_nmea = 0; } break;
      case 4: // check proper length
        if(msg->data.at(i) == 28){ count = 1; decode_nmea = 4; }
        else if(msg->data.at(i) == 0){
          gps_rtk.time = gps_rtk.latitude = gps_rtk.longitude = gps_rtk.altitude =
          gps_rtk.amsl = gps_rtk.hacc = gps_rtk.vacc = count = 0;
          decode_nmea = 5;
          decode_llh = 0;
        }
        else{ decode_nmea = 0; }
        break;
      case 5: // parse llh data
        switch(decode_llh){
          case 0: // time of week data
            ROS_INFO("count: %d, time: %f",count,gps_rtk.time);
            gps_rtk.time += msg->data.at(i) << count*8; break;
          case 1: // longitude
            ROS_INFO("count: %d, longitude: %f",count,gps_rtk.longitude);
            gps_rtk.longitude += (255 - msg->data.at(i)) << count*8; break;
          case 2: // latitude
            ROS_INFO("count: %d, latitude: %f",count,gps_rtk.latitude);
            gps_rtk.latitude += msg->data.at(i) << count*8; break;
          case 3: // altitude -- ellipsoidal
            ROS_INFO("count: %d, altitude: %f",count,gps_rtk.altitude);
            gps_rtk.altitude += msg->data.at(i) << count*8; break;
          case 4: // altitude -- mean sea level
            ROS_INFO("count: %d, amsl: %f",count,gps_rtk.amsl);
            gps_rtk.amsl += msg->data.at(i) << count*8; break;
          case 5: // horizontal accuracy
            gps_rtk.hacc += msg->data.at(i) << count*8; break;
          case 6: // vertical accuracy
            gps_rtk.vacc += msg->data.at(i) << count*8; break;
          default: //  publish gps_rtk while processing msg->data.at(i) = checksum(1)
            // convert output to degrees or meters respectively
            gps_rtk.longitude *= -1.0/10000000;
            gps_rtk.latitude /= 10000000;
            gps_rtk.altitude /= 1000;
            gps_rtk.amsl /= 1000;
            gps_rtk.hacc /= 1000;
            gps_rtk.vacc /= 1000;
            gps_rtk.header.stamp = ros::Time::now();
            reach_pub.publish(gps_rtk);
            decode_nmea = 0;
            break;
        }
        count++;
        if(count == 4){ decode_llh++; count = 0; }
        break;
      default: ROS_WARN(" state: %d should not exist",decode_nmea); break;
    }
  }
}
