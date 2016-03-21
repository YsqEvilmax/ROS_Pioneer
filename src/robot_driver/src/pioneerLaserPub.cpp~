#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

extern geometry_msgs::Twist velocityCommand;

int main(int argc, char **argv)
{
  init(argc,argv, "pioneer_laser_pub");
  NodeHandle handle;
  Publisher pub_object = handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
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
