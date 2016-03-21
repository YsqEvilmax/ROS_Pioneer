#!/bin/bash

#defualt parametres 
master_url="http://localhost:11311"
ros_ip="localhost"

pkg_name="robot_driver"
launcher="launchStageLaser.launch"

#parse the cmd
function usage
{
    echo >&2 "usage: $0 [-h] [-m master_url] [-r ros_ip] [-p pkg_name] [-l launcher]"
}

while getopts h?pl: opt
do
    case "$opt" in
      m)  master_url="$OPTARG";;
      r)  ros_ip="$OPTARG";;
      p)  pkg_name="$OPTARG";;
      l)  launcher="$OPTARG";;
      h|\?)
          usage
	  exit 1;;
    esac
done
shift `expr $OPTIND - 1`

#The ROS_MASTER_URI and ROS_IP/ ROS_HOSTNAME will need to be set for ROS to run and/or for communication with the robot laptop. 
export ROS_MASTER_URI=$master_url
export ROS_IP=$ros_ip

#cmd execution
catkin_make
source devel/setup.bash
roslaunch "$pkg_name" "$launcher"
