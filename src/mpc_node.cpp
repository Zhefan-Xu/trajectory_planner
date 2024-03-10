#include <ros/ros.h>
#include <trajectory_planner/mpcPlanner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;

	trajPlanner::mpcPlanner mp;
	mp.makePlan();

	return 0;
}