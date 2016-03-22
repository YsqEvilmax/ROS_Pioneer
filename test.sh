#!/bin/bash

#defualt parametres 
master_url="http://localhost:11311"
ros_ip="localhost"

pkg_name="robot_driver"
launcher="launchStageLaser.launch"

#parse the cmd
function usage
{
    echo >&2 "usage: $0 [-h] [-i] [-m master_url] [-r ros_ip] [-p pkg_name] [-l launcher]"
}

function init
{
    if ["$ROS_ROOT" = ""]; then
        export ROS_ROOT="/opt/ros/indigo/share/ros"
    fi
    
    source $ROS_ROOT/../../setup.bash
    cd src
    catkin_init_workspace
    catkin_create_pkg "$pkg_name" std_msgs roscpp
    cd ..
    source devel/setup.bash
}

while getopts mrplih?: opt
do
    case "$opt" in
      m)  master_url="$OPTARG";;
      r)  ros_ip="$OPTARG";;
      p)  pkg_name="$OPTARG";;
      l)  launcher="$OPTARG";;
      i)  init;;
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
