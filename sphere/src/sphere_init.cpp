/*
 * FILE: sphere_init.cpp
 * AUTHOR: Wiliiam Willie Wells
 * DATE: April - May 2015
 */

// c++ include
#include <cmath>

// class include
#include "../include/sphere/sphere.h"

// initialize parameters
void Sphere::initParams(){
    // initialize booleans
    ready = false;
    stop = false;
    srv = true;
    center = true;
    change_state = true;
    launched = false;
    idle = false;
    object_pose_known = false;
    arrived = false;
    first = true;
    last = false;

    // center of rotation, verify inbounds
    pnh.param("zCenter",center_offset_z,1.3);
    pnh.param("xCenter",center_offset_x,0.0);
    pnh.param("yCenter",center_offset_y,0.0);
    if(center_offset_z < 1.1 || center_offset_z > 1.5){ 
        center_offset_z = 1.3; 
    }
    if(center_offset_x < -0.9 || center_offset_x > 0.9){ 
        center_offset_x = 0.0; 
    }
    if(center_offset_y < -0.9 || center_offset_y > 0.9){
        center_offset_y = 0.0;
    }

    // launch height and speed
    pnh.param("launch_height",launchZ,0.7);
    if(launchZ < 0.1 || launchZ > 2.0){ launchZ = 0.7; }
    pnh.param("launch_speed",launchV,0.07);
    if(launchV < 0.02){ launchV = 0.07; }

    // radius and arc speed
    pnh.param("radian",radian,16.0);
    if(radian < 4.0 || radian > 64.0){ radian = 16.0; }
    pnh.param("radius", constrict,1.0);
    if(constrict > 1.0 || constrict < 0.65){ constrict = 0.7; }
    pnh.param("contract_rate",constrict_step,0.07);
    if(constrict_step > 0.09 || constrict_step < 0.05){ constrict_step = 0.07; }
    pnh.param("velocity",velocity,1.6);
    if(fabs(velocity) > 1.6 || fabs(velocity) < 0.1){ velocity = 1.4; }

    // amount of revolutions
    pnh.param("rev",revs,3); 
    if(revs < 1 || revs > 12){ revs = 3; }

    // yaw speed
    pnh.param("rotate_speed",WV,0.0);

    // sphere or spiral mode
    pnh.param("mode",s_mode,false);

    // object name
    pnh.param("name",object_name,string("Tripod"));

    // test mode 
    pnh.param("test_mode",test,false);

    // initialize real numbers
    direction = -1;
    theta = 0.0;
    initialX = 0.1;
    initialY = 0.3;
    object_distance = 0.0;
    distance_from = 0.0;
    rotate = PI/radian;
}

// initialize subscribers
void Sphere::initSubscribers(){
    // subscribe to tripod position
    object_sub = nh.subscribe("/vicon/" + object_name + "/" + object_name,10,&Sphere::callbackObject,this);
    // subscribe to position
    spryt_sub = nh.subscribe<collab_msgs::PoseEulerVel>("subject_pose_vel",10,&Sphere::callbackPose,this);
    // subscribe to status
    status_sub = nh.subscribe<pv_ctrl::PVStatus>("pv_hybrid_status",10,&Sphere::callbackStatus,this);
}

// initialize publishers
void Sphere::initPublishers(){
    // publish subject pose velocity
    pose_pub = nh.advertise<collab_msgs::PoseEulerVel>("trajectory_pose",10,true);
}

// initialize services
void Sphere::initServices(){
    // initiate position and state service client
    srv_pose = nh.serviceClient<pv_ctrl::PVCommand>("move_command");
    // initiate landing service
    srv_state = nh.serviceClient<pv_ctrl::PVPidCommand>("pid_stick_cmd");
}

