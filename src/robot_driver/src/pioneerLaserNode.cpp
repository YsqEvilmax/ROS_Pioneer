#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace ros;

geometry_msgs::Twist velocityCommand;

pair<float, float> CheckBlock(const sensor_msgs::LaserScan::ConstPtr &laserScanData, int start, int end)
{
    float min = 99999, max = 0;
    for(int i = start; i < end; i++)
    {
        if(laserScanData->ranges[i] < min)
        {min = laserScanData->ranges[i];}
        if(laserScanData->ranges[i] > max)
        {max = laserScanData->ranges[i];}
    }
    return make_pair(min, max);
}
void SpeedHandler(float space)
{
    if(space < 0.15) // reverse
    {
        cout<< "Watch out! Reverse first." << endl;
        velocityCommand.linear.x = -0.1;
    }
    else if(space < 0.35)
    {
        cout<< "Too close! Speed down." << endl;
        velocityCommand.linear.x = 0;
    }
    else
    {
        velocityCommand.linear.x = 0.1;
    }
}

void BalanceKeeper(float left, float right)
{
     if(left >= right)
     {
        cout << "turn left!" << endl;
        velocityCommand.angular.z = 0.1;
     }
     else
     {
        cout << "turn right!" << endl;
        velocityCommand.angular.z = -0.1;
     }      
}

void Go(const sensor_msgs::LaserScan::ConstPtr &laserScanData)
{
    float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);
    int blockDataNum = rangeDataNum / 3;

    pair<float, float> rightBlock = CheckBlock(laserScanData, 0, blockDataNum);
    pair<float, float> midBlock = CheckBlock(laserScanData, blockDataNum, blockDataNum *2);
    pair<float, float> leftBlock = CheckBlock(laserScanData, blockDataNum * 2,  rangeDataNum);
   
    SpeedHandler(midBlock.first);
    BalanceKeeper(leftBlock.first, rightBlock.first);

    cout << "left_min: " << leftBlock.first << "mid_min: " << midBlock.first << "right_min: " << rightBlock.first << endl;
    cout << "left_max: " << leftBlock.second << "mid_max: " << midBlock.second << "right_max: " << rightBlock.second << endl;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
    Go(laserScanData);   
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
