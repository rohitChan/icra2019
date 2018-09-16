#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdexcept> //for range_error
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <complex>
#include <cmath>
#include <vector>
#include <cstddef>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/dq_controller.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <dq_robotics/IkHand.h>
#include <ros/package.h>

ros::Publisher jointState_pub;
sensor_msgs::JointState ar10_JointState;

void ar10_jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (int j=0; j < ar10_JointState.name.size(); j++)
	{
		// std::cout << "joint_names[" << j << "]: " << joint_names[j]  << std::endl;
		for (int i=0; i < msg->name.size(); i++)
		{
			if(msg->name[i].compare(ar10_JointState.name[j])==0)
			{
				ar10_JointState.position[j]=msg->position[i];	
			}
		}
	}
	jointState_pub.publish(ar10_JointState);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_ar10_kinematics");
	ros::NodeHandle n;

	ar10_JointState.name.clear();
	ar10_JointState.name.push_back("servo0");
	ar10_JointState.name.push_back("servo1");
	ar10_JointState.name.push_back("servo8");
	ar10_JointState.name.push_back("servo9");

	ar10_JointState.position.clear();
	ar10_JointState.position.push_back(0.896);
	ar10_JointState.position.push_back(0.303);
	ar10_JointState.position.push_back(0.134);
	ar10_JointState.position.push_back(0.499);

  jointState_pub = n.advertise<sensor_msgs::JointState>("/ar10/right/servo_positions", 1000);
	ros::Subscriber sub = n.subscribe("/ar10_right/ar10/right/joint_states", 1000, ar10_jointStateCallback);
	ros::Rate hz(100);
	while(ros::ok())
	{
		jointState_pub.publish(ar10_JointState);
		ros::spinOnce();
		hz.sleep();	
	}
	return 0;
}
