#include <iostream>
#include <cmath>
#include <stack>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace ros;

geometry_msgs::Twist velocityCommand;

//the Robot controller class
class RobotController
{
private:
	static const int blockNum = 3;//The number that the laser ranges are divided
	static const float dangerDis = 0.15;//Reserve when distance is smaller than this.
	static const float auctionDis = 0.35;//Slow down when distance is smaller than this.
	static const float trimDis = 0.5;//The range longer than this is meaningless.
	static const int conflictTole = 3;//The tolerance of choice conflict.
	static const int loop90 = 38;//The loop number to turn around.
	int conflictCount;//The choice conflict counter.
 	int loopCount;//The turn around loop counter.
	std::stack<float> trace;//The choice stack.
public:
    RobotController()
    {
        this->conflictCount = 0;
        this->loopCount = 0;
    }

	void Go(const sensor_msgs::LaserScan::ConstPtr &laserScanData)
	{
	    float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);
	    int blockDataNum = rangeDataNum / blockNum;//How many ranges in one block

	    pair<float, float> rightBlock = CheckBlock(laserScanData, 0, blockDataNum);//right block
	    pair<float, float> midBlock = CheckBlock(laserScanData, blockDataNum, blockDataNum *2);//mid block
	    pair<float, float> leftBlock = CheckBlock(laserScanData, blockDataNum * 2,  rangeDataNum);//left block

	    //control speed based on the minimum length in mid block
	    SpeedHandler(midBlock.first);
	    //keep balance by compare the minmum length in left and right block
	    BalanceKeeper(leftBlock.first, rightBlock.first);
	    //escape from a "head-shaking" trap
	    TrapEscaper(velocityCommand.angular.z, midBlock.first);

	    //imformation display
	    cout << "left_min: " << leftBlock.first << " "
	    	 << "mid_min: " << midBlock.first << " "
			 << "right_min: " << rightBlock.first << endl;
	    cout << "left_max: " << leftBlock.second << " "
	    	 << "mid_max: " << midBlock.second << " "
			 << "right_max: " << rightBlock.second << endl;
	    if(!trace.empty())
	    	cout << trace.top() << endl;

	}

protected:
	void SpeedHandler(float space)
	{
	    if(space < dangerDis)//Potentially knock ahead, reverse first.
	    {
	        cout<< "Watch out! Reverse first." << endl;
	        velocityCommand.linear.x = -0.1;
	    }
	    else if(space < auctionDis)//Too close to the obstacle in front, slow down
	    {
	        cout<< "Too close! Speed down." << endl;
	        velocityCommand.linear.x = 0;
	    }
	    else//no problem, go ahead
	    {
	        velocityCommand.linear.x = 0.1;
	    }
	}

	void BalanceKeeper(float left, float right)
	{
	     if(left > right)//left space is wider
	     {
	        cout << "Turn left!" << endl;
	        velocityCommand.angular.z = 0.1;
	     }
	     else if(left < right)//right space is wider
	     {
	        cout << "Turn right!" << endl;
	        velocityCommand.angular.z = -0.1;
	     }
	     else//balanced
	     {
		    cout << "Go straight!" << endl;
		    velocityCommand.angular.z = 0;
	     }
	}

	void TrapEscaper(float value, float frontDis)
	{
		if(value != 0)
		{
			if(!trace.empty())
			{
				//turn left and then right
				if(trace.top() + value == 0 && frontDis <= auctionDis)
				{
					cout << conflictCount << " head shake is detected!" << endl;
					conflictCount++;
				}
			}
			trace.push(value);
		}
		if(loopCount >= loop90)//have been turning for loopCount times
		{
			cout << "Have turn 90 degree!" << endl;
			//clear choice conflict traces
			for(int i = 0; i< conflictCount; i++)
			{
				trace.pop();
			}
			//reset conflictCount and loopCount
			conflictCount = 0;
			loopCount = 0;
		}
		//more than conflictCount conflicts are detected, you are trapped
		if(conflictCount >= conflictTole)
		{
			loopCount++;
            cout << "Trapped, turn around!" << endl;
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = -0.1;
			return;
		}
		return;
	}

private:
	pair<float, float> CheckBlock(const sensor_msgs::LaserScan::ConstPtr &laserScanData, int start, int end)
	{
		//get the min and max lengths in a block
	    float min = 999999, max = 0;
	    for(int i = start; i < end; i++)
	    {
	        if(laserScanData->ranges[i] < min)
	        {min = laserScanData->ranges[i];}
	        if(laserScanData->ranges[i] > max)
	        {max = laserScanData->ranges[i];}
	    }
	    //return the trimmed result
	    return make_pair(min > trimDis ? trimDis : min, max);
	}
};

//the robot controller instance
RobotController rc;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	//just Go Go Go!
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
