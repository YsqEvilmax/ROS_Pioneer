#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace ros;

geometry_msgs::Twist velocityCommand;

pair<float, float> checkBlock(const sensor_msgs::LaserScan::ConstPtr &laserScanData, int start, int end)
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
void speedHandler(float space)
{
    if(space < 0.15) // reverse
    {
        cout<< "Watch out! Reverse first." << endl;
        velocityCommand.linear.x = -0.1;
    }
    else if(space < 0.3)
    {
        velocityCommand.linear.x = 0;
    }
    else
    {
        velocityCommand.linear.x = 0.1;
    }
}

void keepBalance(const sensor_msgs::LaserScan::ConstPtr &laserScanData)
{
    float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);
    int blockDataNum = rangeDataNum / 3;

    pair<float, float> rightBlock = checkBlock(laserScanData, 0, blockDataNum);
    pair<float, float> midBlock = checkBlock(laserScanData, blockDataNum, blockDataNum *2);
    pair<float, float> leftBlock = checkBlock(laserScanData, blockDataNum * 2,  rangeDataNum);

    if(midBlock.first < 0.4)
    {
        cout << "Too colse to the obticale in front, ";
        speedHandler(midBlock.first);
        //velocityCommand.linear.x = 0;
        if(leftBlock.second >= rightBlock.second)
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

    if(leftBlock.first < 0.3)
    {
        cout<< "Potential knock at left! Turn right!" << endl;
        speedHandler(leftBlock.first);
        //velocityCommand.linear.x = 0.1;
        velocityCommand.angular.z = -0.1;
    }
 
    if(rightBlock.first < 0.3)
    {
        cout<< "Potential knock at right! Turn left!" << endl;
        speedHandler(rightBlock.first);
        //velocityCommand.linear.x = 0.1;
        velocityCommand.angular.z = 0.1;
    }

    cout << "left_min: " << leftBlock.first << "mid_min: " << midBlock.first << "right_min: " << rightBlock.first << endl;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
    velocityCommand.linear.x = 0.1;
    velocityCommand.angular.z = 0;
    keepBalance(laserScanData);   
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
