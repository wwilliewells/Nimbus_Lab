/*
 * FILE: sphere_callback.cpp
 * AUTHOR: Wiliiam Willie Wells
 * DATE: April - May 2015
 */

// c++ includes
#include <cmath>

// class include
#include "../include/sphere/sphere.h" 

// calculate distance of object to virtual cage edge
void Sphere::calcDistance(){
    if(2.0 - object_pose.transform.translation.x <= object_pose.transform.translation.x + 2.0){
        object_distance = 2.0 - object_pose.transform.translation.x;
    }else{ object_distance = object_pose.transform.translation.x + 2.0; }
    if(2.0 - object_pose.transform.translation.y <= object_pose.transform.translation.y + 2.0){
        if(object_distance > 2.0 - object_pose.transform.translation.y){
            object_distance = 2.0 - object_pose.transform.translation.y;
        }
    }else{
        if(object_distance > object_pose.transform.translation.y + 2.0){
            object_distance = object_pose.transform.translation.y + 2.0;
        }
    }
}

// callback for status change
void Sphere::callbackStatus(const boost::shared_ptr<pv_ctrl::PVStatus const> &msg){
    mel.lock();
  
    status_srv = (*msg);
    // if publishing is available or moving and not landing set pose
    if( (srv == true || spryt_srv.request.command == 3) && stop == false){
        // hybrid state
        if(status_srv.status == 1){// OFF
            if(spryt_srv.request.command == 1){ change_state = true; }
        }else if(status_srv.status == 4){// idle
            if(spryt_srv.request.command == 0){ // OFF
                change_state = true;
            }else if(spryt_srv.request.command == 3){ // MOVE
                change_state = true; 
            }
        }else if(status_srv.status == 5){ // hovering 
            if(spryt_srv.request.command != 0){ change_state = true; }
        }
         
        // set yaw and rotation speed
        if(s_mode == true || test == true){
            spryt_srv.request.yaw = PI;
            pose.rotation.z = PI;
        }else{
            if(initialX < 0.0){ // clockwise rotation
                spryt_srv.request.yaw = theta*rotate + PI/2;
            }else{ // counter clockwise rotation
                spryt_srv.request.yaw = 3*PI/2 - theta*rotate; 
            }
        }
        pose.angular_velocity.x = WV;
        pose.angular_velocity.y = WV;
        pose.angular_velocity.z = WV;

        // set position
        if(status_srv.status <= 4){
            // set initial position and velocity
            spryt_srv.request.pose.x = initialX;
            spryt_srv.request.pose.y = initialY;
            spryt_srv.request.pose.z = launchZ;
            spryt_srv.request.speed = launchV;
        }else{
            // set ready message: same pose?
            if(status_srv.status == 5){ ready = true; }

            // request movement
            spryt_srv.request.command = 3;
            
            // start in and return to center
            if(center == true && s_mode == true){
                spryt_srv.request.pose.x = center_offset_x;
                spryt_srv.request.pose.y = center_offset_y;
                spryt_srv.request.pose.z = center_offset_z;
                spryt_srv.request.speed = 0.15;

                // iterate through rotations and leave center
                direction++;
                center = false;
            }else if(center == true && s_mode == false){ // spiral initial position
                spryt_srv.request.pose.x = initialX;
                spryt_srv.request.pose.y = initialY;
                spryt_srv.request.pose.z = launchZ;
                spryt_srv.request.speed = 0.05;
                
                direction++; // set to 0, leave initial point
                center = false;
            }else{
                // rate of arc change
                if((s_mode == true || arrived == true) && status_srv.status == 5){
                    // change target position if hovering and in performance position
                    if(theta < 2*radian){ theta++; } // 0 --> 2*PI
                    else{ 
                        theta = 0.0; // reset angle to 0
                        if(s_mode == true){ center = true; } // sphere mode return to center
                        else{ // spiral mode decrease amount of points visited
                            if(radian > 6){ radian -= 4;  rotate = PI/radian; }
                            direction++; 
                        }
                    }
                }

                // set moving position
                spryt_srv.request.pose.x = center_offset_x;
                spryt_srv.request.pose.y = center_offset_y;
                spryt_srv.request.pose.z = center_offset_z;
                if(s_mode == true){
                    if(direction == 6*revs){ // Land 
                        stop = true;
                        ROS_INFO("Landing"); 
                    }else if(direction < revs){ // line in x 
                        spryt_srv.request.pose.x += constrict*sin(theta*rotate); 
                    }else if(direction >= revs && direction < 2*revs){ // line in y
                        spryt_srv.request.pose.y += constrict*sin(theta*rotate);
                    }else if(direction >= 2*revs && direction < 3*revs){ // line in z
                        spryt_srv.request.pose.z += constrict*sin(theta*rotate);
                    }else if(direction >= 3*revs && direction < 4*revs){ // circle in x and y
                        spryt_srv.request.pose.x += constrict*cos(theta*rotate);
                        spryt_srv.request.pose.y += constrict*sin(theta*rotate);
                    }else if(direction >= 4*revs && direction < 5*revs){ // circle in y and z
                        spryt_srv.request.pose.z += constrict*cos(theta*rotate);
                        spryt_srv.request.pose.y += constrict*sin(theta*rotate);
                    }else if(direction >= 5*revs && direction < 6*revs){ // circle in x and z
                        spryt_srv.request.pose.x += constrict*cos(theta*rotate);
                        spryt_srv.request.pose.z += constrict*sin(theta*rotate);
                    }
                    
                    // vary velocity with cosine
                    spryt_srv.request.speed = cos(theta*rotate);
                }else{ // if implementing a spiral, pose is known, and launch procedure done
                    if(object_pose_known && launched){
                        if(!arrived){
                            // approach spiral radius
                            if(object_distance >= constrict){ 
                                distance_from = object_distance - 0.05;

                                // move to closest linear radius position
                                if(initialX < 0.0){
                                    spryt_srv.request.pose.x -= distance_from;
                                }else{ 
                                    spryt_srv.request.pose.x += distance_from;
                                }

                                // set velocity of approach to maximum
                                spryt_srv.request.speed = 1.0;

                                // indicate moved to radius position
                                arrived = true; ROS_INFO("Arrived");
                            }else{ stop = true; object_pose_known = false; }
                        }else{
                            // set x position 
                            if(initialX < 0.0){ 
                                spryt_srv.request.pose.x -= distance_from*cos(theta*rotate);
                            }else{
                                spryt_srv.request.pose.x += distance_from*cos(theta*rotate);
                            }

                            // set y position
                            spryt_srv.request.pose.y += distance_from*sin(theta*rotate);

                            // contract radius up to a point
                            if(direction == revs && distance_from > constrict){
                                distance_from -= constrict_step;
                                direction = 0;
                            }

                            // if radius is too small land
                            if(distance_from <= constrict){
                                spryt_srv.request.pose.x = initialX;
                                spryt_srv.request.pose.y = initialY;
                                spryt_srv.request.pose.z = center_offset_z;
                                stop = true;
                            }

                            // vary height with sine
                            spryt_srv.request.pose.z += 0.5*sin(theta*rotate); 
                         
                            // travel at max velocity
                            spryt_srv.request.speed = velocity;
                        }
                    }else{ // object position is unknown or UAV has not launched, target
                        spryt_srv.request.pose.x = initialX;
                        spryt_srv.request.pose.y = initialY;
                        spryt_srv.request.pose.z = center_offset_z;
                        spryt_srv.request.speed = 0.5;
                    }
                }
            }
        }

        // set new subject pose velocity
        pose.linear_velocity.x = spryt_srv.request.speed;
        pose.linear_velocity.y = pose.linear_velocity.x;
        pose.linear_velocity.z = pose.linear_velocity.x;
        pose.translation = spryt_srv.request.pose;
        
        // publishing condition
        srv = false;
    }
        
    mel.unlock();
}

// pose callback
void Sphere::callbackPose(const boost::shared_ptr<collab_msgs::PoseEulerVel const> &msg){
    // called before status callback once, a position is available
    if(srv == true && ready == false){ pose = (*msg); ready = true; }

    // obtain initial x and y position
    if(first){ initialX = pose.translation.x; initialY = pose.translation.y; first = false; }
}

// object callback
void Sphere::callbackObject(const boost::shared_ptr<geometry_msgs::TransformStamped const> &msg){
    // only need to calculate and save rotation distance from object once
    object_pose = (*msg);
    if(object_pose_known == false && stop == false){
        center_offset_x = object_pose.transform.translation.x;
        center_offset_y = object_pose.transform.translation.y;

        object_pose_known = true;
        calcDistance();
    }
}

