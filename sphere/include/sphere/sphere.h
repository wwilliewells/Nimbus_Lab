/*
 * FILE: sphere.h
 * AUTHOR: William Willie Wells
 * DATE: April - May 2015 
 */

#ifndef _SPHERE_H_
#define _SPHERE_H_

// ros includes
#include "ros/ros.h"

// mit_asctec includes
#include <pv_ctrl/PVStatus.h>
#include <pv_ctrl/PVCommand.h>
#include <pv_ctrl/PVPidCommand.h>
#include <collab_msgs/PoseEulerVel.h>
#include <geometry_msgs/TransformStamped.h>

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
#define PI 3.1415926535

// class
class Sphere{
    public:
        // con(de)structors
        Sphere();
        ~Sphere();

    private:
        // callbacks
        void callbackPose(const boost::shared_ptr<collab_msgs::PoseEulerVel const>&);
        void callbackStatus(const boost::shared_ptr<pv_ctrl::PVStatus const> &);
        void callbackObject(const boost:: shared_ptr<geometry_msgs::TransformStamped const>&);

        // initializers
        void initParams();
        void initPublishers();
        void initSubscribers();
        void initServices();

        // start thread 
        void startThread(bool);

        // stop thread
        void stopThread(bool);

        // active threads
        void sphereThread();
        void poseStateThread();

        // state change
        void launch();
        void land();
        void sleep();

        // calculate distance to an object
        void calcDistance();

        // ros objects
        ros::NodeHandle nh,pnh;

        // ros publisher
        ros::Publisher status_pub,pose_pub;

        // ros subscribers
        ros::Subscriber status_sub,spryt_sub,object_sub;

        // ros services
        ros::ServiceClient srv_state, srv_pose;

        // thread objects
        boost::mutex mel;
        boost::shared_ptr<boost::thread> hybrid_thread,sphere_thread;

        // position and state service objects
        pv_ctrl::PVCommand spryt_srv;
        pv_ctrl::PVPidCommand land_srv;

        // position and status messages
        pv_ctrl::PVStatus status_srv;
        collab_msgs::PoseEulerVel pose;
        geometry_msgs::TransformStamped object_pose;

        // volatile flags
        volatile bool ready;        
        volatile bool stop;
        volatile bool srv;
        volatile bool center;
        volatile bool change_state;
        volatile bool launched;
        volatile bool idle;
        bool object_pose_known;
        bool arrived;
        bool first;
        bool last;
   
        // truly private variables
        volatile double theta;
        volatile double initialX,initialY;
        volatile double object_distance,distance_from;
        int direction;
        double rotate;

        // user specifiable variables
        double center_offset_x;
        double center_offset_y;
        double center_offset_z;
        double launchZ;
        double launchV;
        double WV;
        double radian;
        double constrict;
        double constrict_step;
        double velocity;
        int revs;
        bool s_mode;
        bool test;
        string object_name;
};

#endif // sphere
