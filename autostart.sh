# !/bin/bash

set -e
{
source /opt/ros/jade/setup.bash
source /home/wwilliewells/ros/gps_testing/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311/
export ROS_PACKAGE_PATH=/home/wwilliewells/ros:/opt/ros/jade/share:/opt/ros/jade/stacks

sleep 5

}

set -v
{
nohup roslaunch gps_testing rf_testing
}
