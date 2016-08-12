/*
 * FILE: sphere_thread.cpp
 * AUTHOR: Wiliiam Willie Wells
 * DATE: April - May 2015
 */

// c++ includes
#include <cmath>

// class include
#include "../include/sphere/sphere.h" 

// unlock mutual exclusion to publish, sleep, and relock
void Sphere::sleep(){
    mel.unlock();
    ros::Duration(3.0).sleep();
    mel.lock();
}

// launch UAV
void Sphere::launch(){
    mel.lock();

    // launching: do not need to land
    stop = false;

    // transition to service MOTORS_ON 
    spryt_srv.request.command = 0;
    sleep();
    if(status_srv.status == 1){ change_state = true; }
    
    // transition to service MOTORS_IDLE 
    spryt_srv.request.command = 1;
    sleep();
    if(status_srv.status == 4){ change_state = true; }
        
    // transition to service MOTORS_MOVE
    while(status_srv.status == 4){
        spryt_srv.request.command = 3;
        sleep(); 
        if(status_srv.status == 5 || status_srv.status == 6){ 
            launched = true; 
            ROS_INFO("Launched"); 
        }
    }

    mel.unlock();
}

// land UAV
void Sphere::land(){
    mel.lock();

    for(int i=0;i<4;i++){
        ready = true;
        change_state = true;
        // transition to low height
        if(i < 2){
            spryt_srv.request.command = 3;
            if(i == 1){ spryt_srv.request.pose.z = 0.2; }
            else{ spryt_srv.request.pose.z = center_offset_z; }
            spryt_srv.request.pose.x = initialX;
            spryt_srv.request.pose.y = initialY;
            spryt_srv.request.yaw = PI;
            spryt_srv.request.speed = 1.0;
            pose.translation = spryt_srv.request.pose;
            pose.rotation.z = PI;
        }else if(i == 2){
            // transition to MOTORS_IDLE = 2
    	    idle = true;
            land_srv.request.command = 2;
        }else if(i == 3){
            // transition to MOTORS_OFF = 1
           land_srv.request.command = 1;
        }
        sleep();
    }
    
    // force motors off
    while(status_srv.status != 1){
        ready = true;
        change_state = true;
        land_srv.request.command = 1;
        sleep();
    }

    // landing a success
    stop = false;
    mel.unlock();
}

// stop a thread
void Sphere::stopThread(bool t){
     switch(t){
         case(0): hybrid_thread->join(); break;
         case(1): sphere_thread->join(); break;
         default:  break;
     }
}

// start a thread 
void Sphere::startThread(bool t){
    switch(t){
        case(0):
            hybrid_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Sphere::poseStateThread,this))); break;
        case(1):
            sphere_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Sphere::sphereThread,this))); break;
        default: break;
    }
}

// hybrid position state thread
void Sphere::poseStateThread(){
    // set publishing rate
    ros::Rate pub_rate(20);
    // publish state and position data
    while(ros::ok() && last == false){
        mel.lock();
        // set multiple conditions to publish, 
        // new position and target available, not landing
        if(srv == false && ready == true && stop == false){
            // state change required, moving, or hovering
            if(change_state || (status_srv.status > 4 && spryt_srv.request.command == 3)){
                // publish if status state is < command state or if status is > 3
                if(status_srv.status <= spryt_srv.request.command || status_srv.status == 4 || status_srv.status == 5){
                    //ROS_INFO("X:%f, Y:%f, Z:%f, W:%f, V:%f",spryt_srv.request.pose.x,spryt_srv.request.translation.y,spryt_srv.request.translation.z,spryt_srv.request.yaw,spryt_srv.request.speed);
                    // publish new position and new target
                    pose_pub.publish(pose);
                    srv_pose.call(spryt_srv);
                }
            }

            // reset publishing conditions
            srv = true;
            ready = false;
            change_state = false;

        // landing: publish if a new position and target are available, 
        // state change required
        }else if(srv == false && ready == true && stop == true && change_state == true){
            // publish target and pose, or command land sequence
            if(idle == false){ pose_pub.publish(pose); srv_pose.call(spryt_srv);  }
            else{ srv_state.call(land_srv); }
        }

        // unlock and sleep
        mel.unlock();
        pub_rate.sleep();
    }
}
       
// main thread
void Sphere::sphereThread(){
    // start hybrid thread
    startThread(0);

    // main thread launchs, relies on callbacks, lands
    while(ros::ok() && last == false){
        // launch UAV
        if(launched == false){ launch(); }
        
        // land UAV
        if(stop == true){ land(); stop = false; last = true; }
    }
    
    // stop hybrid thread
    stopThread(0);
}

