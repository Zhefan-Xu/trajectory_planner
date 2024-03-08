#include <trajectory_planner/clustering/obstacleClustering.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/utils.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout; using std::endl;
ros::Publisher currPoseVisPub;
visualization_msgs::Marker currPoseMarker;
bool initCurrPose = false;


bool newMsg = false;
std::vector<double> newPoint {0, 0, 1.0, 0};
void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
	newPoint[0] = cp->pose.position.x;
	newPoint[1] = cp->pose.position.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newPoint[3] = trajPlanner::rpy_from_quaternion(cp->pose.orientation);
	newMsg = true;
}


void publishCurrPoseVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (initCurrPose){
			currPoseVisPub.publish(currPoseMarker);
		}
		r.sleep();
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "test_obstacle_clustering");
	ros::NodeHandle nh;

	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	currPoseVisPub = nh.advertise<visualization_msgs::Marker>("/current_pose", 1000);
	std::thread startVisWorker = std::thread(publishCurrPoseVis);

	obstacleClustering oc;
	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Test Clustering Node]: Request No. " << countLoop+1 << endl;
		cout << "[Planner Node]: Wait for current pose point..." << endl;
		while (ros::ok()){
			if (newMsg){
				std::vector<double> currPose = newPoint;
				newMsg = false;
				cout << "[Planner Node]: current pose point OK. (" << currPose[0] << " " << currPose[1] << " " << currPose[2] << " " << currPose[3] << ")" << endl;
				
				// visualization:
				initCurrPose = true;
				currPoseMarker.header.frame_id = "map";
				currPoseMarker.header.stamp = ros::Time();
				currPoseMarker.ns = "currPoseVis";
				currPoseMarker.id = 0;
				currPoseMarker.type = visualization_msgs::Marker::ARROW;
				currPoseMarker.action = visualization_msgs::Marker::ADD;
				currPoseMarker.pose.position.x = newPoint[0];
				currPoseMarker.pose.position.y = newPoint[1];
				currPoseMarker.pose.position.z = newPoint[2];
				currPoseMarker.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, newPoint[3]);
				currPoseMarker.lifetime = ros::Duration(0.5);
				currPoseMarker.scale.x = 0.8;
				currPoseMarker.scale.y = 0.2;
				currPoseMarker.scale.z = 0.4;
				currPoseMarker.color.a = 0.7;
				currPoseMarker.color.r = 1.0;
				currPoseMarker.color.g = 0.5;
				currPoseMarker.color.b = 1.0;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}


		// get local point cloud
		

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}

	ros::spin();
	return 0;
}