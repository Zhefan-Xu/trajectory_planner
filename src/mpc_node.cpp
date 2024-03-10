#include <ros/ros.h>
#include <trajectory_planner/mpcPlanner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "bspline_navigation_node");
	ros::NodeHandle nh;

	mpcPlanner mp ();

	return 0;
}