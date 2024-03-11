#include <ros/ros.h>
#include <trajectory_planner/mpcPlanner.h>
#include <trajectory_planner/polyTrajOccMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;




	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));

	std::shared_ptr<trajPlanner::polyTrajOccMap> pt;
	pt.reset(new trajPlanner::polyTrajOccMap (nh));

	std::shared_ptr<trajPlanner::mpcPlanner> mp;
	mp.reset(new trajPlanner::mpcPlanner (nh));
	mp->makePlan();

	return 0;
}