#!/bin/bash

#defualt parametres 
master_url="http://localhost:11311"
ros_ip="localhost"

pkg_name="robot_driver"
launcher="launchStageLaser.launch"

#initialize the environment variables
if ! [ -n "$ROS_ROOT" ]; then
    export ROS_ROOT="/opt/ros/indigo/share/ros"
fi
source $ROS_ROOT/../../setup.bash

#parse the cmd
function usage
{
    echo >&2 "usage: $0 [-h] [-i] [-c] [-m master_url] [-r ros_ip] [-p pkg_name] [-l launcher]"
}

function init
{
    cd src
    echo "Init work space!"
    catkin_init_workspace

    echo -e 
    echo "Create package $pkg_name!"
    catkin_create_pkg "$pkg_name" std_msgs roscpp

    echo "Modify CMakeLists.txt"
    cd robot_driver
    content=$(<cmake_config.txt)
    (grep -Fxq "$content" CMakeLists.txt) && (echo "Content is already inclued!") || ( echo "Modified :" && echo "$content" && echo "$content" >> "CMakeLists.txt")
    cd ..   
    
    [ -d rosaria ] || git clone https://github.com/amor-ros-pkg/rosaria.git
    cd ..
    source devel/setup.bash
}

function cler
{
    rm -fr ../ros_pioneer_ws
}

while getopts ichm:r:p:l:?: opt
do
    case "$opt" in
      m)  master_url="$OPTARG";;
      r)  ros_ip="$OPTARG";;
      p)  pkg_name="$OPTARG";;
      l)  launcher="$OPTARG";;
      i)  init;;
      c)  cler
          exit 1;;
      h|\?)
          usage
	  exit 1;;
    esac
done
shift `expr $OPTIND - 1`

#The ROS_MASTER_URI and ROS_IP/ ROS_HOSTNAME will need to be set for ROS to run and/or for communication with the robot laptop. 
export ROS_MASTER_URI=$master_url
echo $ROS_MASTER_URI
export ROS_IP=$ros_ip
echo $ROS_IP

#cmd execution
echo "Start to make and launch!"
catkin_make
source devel/setup.bash
roslaunch "$pkg_name" "$launcher"
