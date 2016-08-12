# FILE: generate_node.sh
# AUTHOR: William Willie Wells
# DATE: August 2016 ...

mkdir "src/$1"
path1="src/$1/src"
mkdir "$path1"
path2="src/$1/include"
mkdir "$path2"

#fileH="$1" tr | '[a-z]' '[A-Z]'
#echo "$fileH"
file1="$1".h
echo "/*" > "$path2/$file1"
cat <<HEADER >> $path2/$file1
 * File: $file1
 * AUTHOR: William Willie Wells
 * DATE: $2 $3 ...
 */

#ifndef _'$1'_H_
#define _"$1"_H_

// ros includes
#include <ros/ros.h>
//#include <std_msgs/UInt8MultiArray.h>

// package message include
//#include 

// c++ includes
#include <boost/thread/thread.hpp>

// namespace
using namespace std;

// defines
//#define PI

// class
class $1{
  public:
    // con(de)structors
    $1();
    ~$1();
  private:
    void callback$1(const boost::shared_ptr< const>&);
    void timer$1(const ros::TimerEvent&);

    // initializers
    void initParams();
    void initPublishers();
    void initSubscribers();
    void initTimers();

    // ros objects
    ros::NodeHandle nh, pnh;
    // ros publisher
    ros::Publisher _pub;
    // ros subscribers
    ros::Subscriber _sub;
    ros::Timer timer1;

    // Mutual Exclusion Lock
    boost::mutex mel;

    // class messages

    // in(ex)ternal parameters
};
#endif // $1
HEADER

file2="$1".cpp
echo "/*" > "$path1/$file2"
cat <<MAIN >> $path1/$file2
 * FILE: $file2
 * AUTHOR: William Willie Wells
 * DATE: $2 $3 ...
 */

// class include
#include "../include/$file1"

// defines
#define ROS_NODE "$1"

// constructor
$1::$1():nh(),pnh("~"){
  // initialize parameters
  initParams();

  // initialize publishers
  initPublishers();

  // initialize subscribers
  initSubscribers();

  // initialize timers
  initTimers();
}

// destructor
$1::~$1(){}

// c++ Main
int main(int argc, char **argv){
  // initialize ros node handle
  ros::init(argc,argv,ROS_NODE);

  // log start
  ROS_INFO("Started node %s",ROS_NODE);

  // create and start node instance
  $1 name;

  // keep spinning until landing acheived
  ros::spin();

  // log stop and shutdown
  ROS_INFO("Stopped node %s", ROS_NODE);

  return 0;
}
MAIN

file3="$1"_init.cpp
echo "/*" > "$path1/$file3"
cat <<INIT >> $path1/$file3
 * FILE: $file3
 * AUTHOR: William Willie Wells
 * DATE: $2 $3 ...
 */

// class include
#include "../include/$file1"

// initialize parameters
void $1::initParams(){
  //= 0;
  //= false;
  // set 
  //pnh.param(" ", ,23);
  //ROS_INFO(": %d", );
}

// initialize subscribers
void $1::initSubscribers(){
  // subscribe to
  _sub = nh.subscribe<::>(" ",1000,&$1::callback$1,this);
}

// initialize publishers
void $1::initPublishers(){
  // publish to 
  _pub = nh.advertise<::>(" ",1000,true);
}

// initialize timers
void $1::initTimers(){
  timer1 = nh.createTimer(ros::Duration(0.25),&$1::timer$1,this);
}
INIT

file4="$1"_callback.cpp
echo "/*" > "$path1/$file4"
cat <<CALLBACK >> $path1/$file4
 * FILE: $file4
 * AUTHOR: William Willie Wells
 * DATE: $2 $3 ...
 */

// class include
#include "../include/$file1"

// 
void $1::callback$1(const boost::shared_ptr< const>& msg){
  // 
}
CALLBACK

#THREAD
