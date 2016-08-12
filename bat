# File: bat -- first linux, windows bat file style, executable I created
# Author: William Willie Wells
# Date: May 2015

path1="../bag2csv"
path2="src/bags"
#path3="hummingbird/jenny"
path3="havelock"
mkdir "$path2/csv"
#mkdir "$path2/csv/hummingbird"
#mkdir "$path2/csv/ok_osu"
mkdir "$path2/csv/$path3"

for i in {1..20}
do
    #path4="$path2/csv/$path3/jenny$i"
    path4="$path2/csv/$path3/rf$i"
    mkdir "$path4"

    #bagfile="$path2/$path3/hummingbird_jenny$i.bag"
    bagfile="$path2/$path3/havelock_bag$i.bag"
    #bagfile="$path2/rf_bag$i.bag"

    ./"$path1"/bag_reader.py -b "$bagfile" -t /a/ack -o "$path4"/ack"$i".csv

    ./"$path1"/bag_reader.py -b "$bagfile" -t /a/rf_rx_data -o "$path4"/rf_receive"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/subject_pose -o "$path4"/pose_meters_rfa"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /tardis/subject_pose -o "$path4"/pose_meters_rft"$i".csv

    ./"$path1"/bag_reader.py -b "$bagfile" -t /w/subject_pose -o "$path4"/pose_meters_rfw"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/sensor4_data -o "$path4"/sensor4_hulk_smash"$i".csv

    ./"$path1"/bag_reader.py -b "$bagfile" -t /a/rf_tx_data -o "$path4"/rf_transmit"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/subject_ctrl_state -o "$path4"/state_hulk_smash"$i".csv

    ./"$path1"/bag_reader.py -b "$bagfile" -t /w/subject_gps -o "$path4"/position_rfw"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/robot_gps -o "$path4"/robot_gps_jenny"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /tardis/subject_gps -o "$path4"/position_rf"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /w/subject_gps -o "$path4"/subject_gps_jenny"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /vicon/JENNY/JENNY -o "$path4"/vicon_jenny"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/task_waypose -o "$path4"/task_hulk_smash"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/cmd_subject_ctrl_state -o "$path4"/command_state_hulk_smash"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/robot_status -o "$path4"/robot_status_hulk_smash"$i".csv

    #./"$path1"/bag_reader.py -b "$bagfile" -t /a/robot_tx_data -o "$path4"/control_tx_hulk_smash"$i".csv

   #./"$path1"/bag_reader.py -b "$bagfile" -t /a/device_rx_data -o "$path4"/laser_rx_jenny"$i".csv
done
