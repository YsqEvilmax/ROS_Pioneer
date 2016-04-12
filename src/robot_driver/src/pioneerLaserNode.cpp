#include <iostream>
#include <cmath>
#include <stack>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace ros;

geometry_msgs::Twist velocityCommand;

class RobotController
{
private:
	static const int blockNum = 3;
	static const float dangerDis = 0.15;
	static const float auctionDis = 0.35;
	static const float trimDis = 0.5;
	std::stack<float> trace;
public:
	void Go(const sensor_msgs::LaserScan::ConstPtr &laserScanData)
	{
	    float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);
	    int blockDataNum = rangeDataNum / blockNum;

	    pair<float, float> rightBlock = CheckBlock(laserScanData, 0, blockDataNum);
	    pair<float, float> midBlock = CheckBlock(laserScanData, blockDataNum, blockDataNum *2);
	    pair<float, float> leftBlock = CheckBlock(laserScanData, blockDataNum * 2,  rangeDataNum);

	    SpeedHandler(midBlock.first);
	    BalanceKeeper(leftBlock.first, rightBlock.first);
	    //TrapEscaper(velocityCommand.angular.z, leftBlock.second, rightBlock.second);

	    cout << "left_min: " << leftBlock.first << " "
	    	 << "mid_min: " << midBlock.first << " "
			 << "right_min: " << rightBlock.first << endl;
	    cout << "left_max: " << leftBlock.second << " "
	    	 << "mid_max: " << midBlock.second << " "
			 << "right_max: " << rightBlock.second << endl;
	}

protected:
	void SpeedHandler(float space)
	{
	    if(space < dangerDis) // reverse
	    {
	        cout<< "Watch out! Reverse first." << endl;
	        velocityCommand.linear.x = -0.1;
	    }
	    else if(space < auctionDis)
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
	     if(left > right)
	     {
	        cout << "Turn left!" << endl;
	        velocityCommand.angular.z = 0.1;
	     }
	     else if(left < right)
	     {
	        cout << "Turn right!" << endl;
	        velocityCommand.angular.z = -0.1;
	     }
	     else
	     {
		    cout << "Go straight!" << endl;
		    velocityCommand.angular.z = 0;
	     }
	}

	void TrapEscaper(float value, float leftMax, float rightMax)
	{
		if(!trace.empty()){
			if(trace.top() + value == 0 && value != 0)
			{
				if(velocityCommand.linear.x == 0)
				{
					trace.pop();
			        cout << "Trapped, reverse to escape!" << endl;
			        velocityCommand.linear.x = -0.1;
			        if(leftMax > rightMax)
			        {
			        	velocityCommand.angular.z = -0.1;
			        }
			        if(leftMax < rightMax)
			        {
			        	velocityCommand.angular.z = 0.1;
			        }
			        return;
				}
			}
		}
		trace.push(value);
		return;
	}

private:
	pair<float, float> CheckBlock(const sensor_msgs::LaserScan::ConstPtr &laserScanData, int start, int end)
	{
	    float min = 999999, max = 0;
	    for(int i = start; i < end; i++)
	    {
	        if(laserScanData->ranges[i] < min)
	        {min = laserScanData->ranges[i];}
	        if(laserScanData->ranges[i] > max)
	        {max = laserScanData->ranges[i];}
	    }
	    return make_pair(min > trimDis ? trimDis : min, max);
	}

};

RobotController rc;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
    rc.Go(laserScanData);
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

  return 0;
}
