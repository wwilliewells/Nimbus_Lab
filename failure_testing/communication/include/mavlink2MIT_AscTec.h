/*
 * FILE: mavlink2MIT_AscTec.h
 * AUTHOR: William Willie Wells
 * DATE: August 2015
 */

#ifndef _MAVLINK2MIT_ASCTEC_H_
#define _MAVLINK2MIT_ASCTEC_H_

// ros includes
#include "ros/ros.h"

// mit_asctec includes
#include <collab_msgs/SubjectCtrlState.h>
#include <collab_msgs/AsctecLlStatus.h>
#include <collab_msgs/AsctecImuCalcData.h>
#include <collab_msgs/AsctecGpsData.h>
#include <collab_msgs/AsctecCtrlInput.h>
//#include <collab_msgs/AsctecRotors.h>
#include <collab_msgs/AsctecRcData.h>

// c++ include
#include <boost/thread/thread.hpp>

// package message include
#include <flight_testing/Mavlink.h>
#include <flight_testing/Arducopter.h>

// namespace
using namespace std;

// class
class mmaTranslation{
  public:
    // con(de)structors
    mmaTranslation();
    ~mmaTranslation();

  private:
    // callback from robot
    void callbackMavlinkRx(const boost::shared_ptr<flight_testing::Mavlink const>&);
    // callbacks from mit_asctec
    void callbackState(const boost::shared_ptr<collab_msgs::SubjectCtrlState const>&);
    void callbackControlInput(const boost::shared_ptr<collab_msgs::AsctecCtrlInput const>&);
    //void callbackPoll(const boost::shared_ptr<>&); // may need to send a mavlink poll message
    // callback for timer
    void heartBeat(const ros::TimerEvent&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    void initTimers();

    // threads
    void startThread();
    void stopThread();
    void parsePktTypeThread();

    // additional processes
    void pubGeneral(bool,bool,bool);
    void printBytes();
    uint16_t crc_calc();
    void byte_crc_calc(uint8_t);

    // ros objects
    ros::NodeHandle nh;//, pnh;
    ros::Timer heart_beat;

    // ros publisher
    ros::Publisher not_busy, mit_asctec_imu, mit_asctec_status, mit_asctec_gps, 
      mit_asctec_rc/*, mit_asctec_rotors;*/, mavlink_transmit;

    // ros subscribers
    ros::Subscriber mavlink_receive, control_input, command_state;
       
    // thread object
    boost::mutex mel;
    boost::shared_ptr<boost::thread> parse_thread;//,

    // arducopter messages: transmit, heartbeat, state machine, control input, receive
    flight_testing::Mavlink mavmsg_tx,mavmsg_hb,mavmsg_sm,mavmsg_ci,mavmsg_rx;
    flight_testing::Arducopter enable_launch;
        
    // MIT AscTec messages
    collab_msgs::AsctecLlStatus status;
    collab_msgs::AsctecImuCalcData imu;
    collab_msgs::AsctecGpsData gps;
    collab_msgs::AsctecCtrlInput rci_msg;
    //collab_msgs::AsctecRotors rotors;
    collab_msgs::AsctecRcData remote_control;
    collab_msgs::SubjectCtrlState current_state;

    // private variables
    uint8_t crc8; // not used in current implementation
    uint8_t seq;
    uint8_t mode,custom_mode,previous_mode;
    uint8_t mav_state; 
    uint16_t crc,count,value_rc;
    uint8_t tx_num, byte;
    int value_i, t_pose;
    float value_f;
    uint8_t *bytes;
    vector<uint8_t> value_u;
    bool mavmsg_received;
    double time_start,time_now;
};
#endif // mavlink 2 mit_asctec

