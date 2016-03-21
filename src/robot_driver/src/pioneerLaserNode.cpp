#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace ros;

geometry_msgs::Twist velocityCommand;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);
  velocityCommand.linear.x = 0.1;
  for(int i = 0; i < rangeDataNum; i++)
  {
    if(laserScanData->ranges[i] < 0.5)
    {
      velocityCommand.linear.x = -0.1;
      break;
    }
  }
}

int main(int argc, char **argv)
{
  init(argc,argv, "pioneer_laser_node");
  NodeHandle handle;
  Publisher pub_object = handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
  Subscriber sub_object = handle.subscribe("/scan", 1, laserScanCallback);
  Rate loop_rate(10);


  while(ok())
  {
    spinOnce();
    loop_rate.sleep();
    pub_object.publish(velocityCommand);
  }

  velocityCommand.linear.x = 0;
  pub_object.publish(velocityCommand);
  return 0;
}
