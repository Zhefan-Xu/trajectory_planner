#include <ros/ros.h>
#include <trajectory_planner/mpcPlanner.h>
#include <trajectory_planner/polyTrajOccMap.h>

std::vector<std::vector<double>> waypoints;
bool newMsg = false;
std::vector<double> newPoint {0, 0, 1.0, 0};
void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
	newPoint[0] = cp->pose.position.x;
	newPoint[1] = cp->pose.position.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newPoint[3] = trajPlanner::rpy_from_quaternion(cp->pose.orientation);
	newMsg = true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);

	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));

	std::shared_ptr<trajPlanner::polyTrajOccMap> pt;
	pt.reset(new trajPlanner::polyTrajOccMap (nh));

	std::shared_ptr<trajPlanner::mpcPlanner> mp;
	mp.reset(new trajPlanner::mpcPlanner (nh));


	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Test MPC Node]: Request No. " << countLoop+1 << endl;
		cout << "[Test MPC Node]: Wait for waypoints..." << endl;
		waypoints.clear();
		int countPoints = 0;
		while (ros::ok()){
			std::vector<double> currPose;
			bool lastPoint = false;
			while (ros::ok() and not lastPoint){
				if (newMsg){
					currPose = newPoint;
					newMsg = false;
					if (currPose[3] != 0){
						lastPoint = true;
						cout << "[Test MPC Node]: received last point. Waypoints size: " << waypoints.size() << endl;
						break;
					}
					else{
						cout << "[Test MPC Node]: current point" << " i=" << countPoints << " (" << currPose[0] << " " << currPose[1] << " " << currPose[2] << " " << currPose[3] << ")" << endl;
					}
					waypoints.push_back(currPose);
					++countPoints;
					break;
				}
				ros::spinOnce();
				r.sleep();
			}
		}
	}




	return 0;
}