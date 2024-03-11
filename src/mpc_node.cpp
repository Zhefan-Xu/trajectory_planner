#include <ros/ros.h>
#include <trajectory_planner/mpcPlanner.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher waypointsVisPub;

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

visualization_msgs::MarkerArray waypoints2vis(const std::vector<std::vector<double>>& waypoints){
	visualization_msgs::MarkerArray wpMsg;
	int id = 0;
	for (int i=0; i<int(waypoints.size()); ++i){
		visualization_msgs::Marker wp;
		wp.header.frame_id = "map";
		wp.header.stamp = ros::Time();
		wp.ns = "waypoints";
		wp.id = id;
		wp.type = visualization_msgs::Marker::SPHERE;
		wp.action = visualization_msgs::Marker::ADD;
		wp.pose.position.x = waypoints[i][0];
		wp.pose.position.y = waypoints[i][1];
		wp.pose.position.z = waypoints[i][2];
		wp.lifetime = ros::Duration(0.5);
		wp.scale.x = 0.2;
		wp.scale.y = 0.2;
		wp.scale.z = 0.2;
		wp.color.a = 0.7;
		wp.color.r = 1.0;
		wp.color.g = 0.5;
		wp.color.b = 1.0;		
		++id;
		wpMsg.markers.push_back(wp);
	}
	return wpMsg;
}

void visPub(){
	ros::Rate r(10);
	while (ros::ok()){
		if (waypoints.size() != 0){
			visualization_msgs::MarkerArray wpMsg = waypoints2vis(waypoints);
			waypointsVisPub.publish(wpMsg);
		}
		r.sleep();
	}	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	waypointsVisPub = nh.advertise<visualization_msgs::MarkerArray>("/waypoints", 1000);

	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));

	std::shared_ptr<trajPlanner::polyTrajOccMap> pt;
	pt.reset(new trajPlanner::polyTrajOccMap (nh));

	std::shared_ptr<trajPlanner::mpcPlanner> mp;
	mp.reset(new trajPlanner::mpcPlanner (nh));

	std::thread visWorker = std::thread(visPub);

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Test MPC Node]: Request No. " << countLoop+1 << endl;
		cout << "[Test MPC Node]: Wait for waypoints..." << endl;
		waypoints.clear();
		int countPoints = 0;
		bool lastPoint = false;
		while (ros::ok() and not lastPoint){
			std::vector<double> currPose;
			while (ros::ok() ){
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