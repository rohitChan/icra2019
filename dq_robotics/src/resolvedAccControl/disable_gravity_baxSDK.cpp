#include "ros/ros.h"
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "disble_gravityFromBaxSDK");
	ros::NodeHandle n;
	std_msgs::Empty myMsg;
	ros::Rate loop_rate(10);
	ros::Publisher disble_gravity_pub = n.advertise<std_msgs::Empty>("/robot/limb/right/suppress_gravity_compensation", 1000);
	while(ros::ok())
	{
		disble_gravity_pub.publish(myMsg);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}