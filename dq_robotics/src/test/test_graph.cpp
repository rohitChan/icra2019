#include <vector>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/baxter_poseControl_server.h>
#include "dq_robotics/graph.cpp"
using namespace std;

int main(int argc,char **argv) 
{
  ros::init(argc, argv, "graph_test");
  ros::NodeHandle n;
	plot p;
	for(int a=0;a<100;a++) {
		vector<float> x,y;
		for(int k=a;k<a+200;k++) {
			x.push_back(k);
			y.push_back(k*k);
		}
		p.plot_data(x,y);
	}
	while(true)
	{}
	return 0;
}