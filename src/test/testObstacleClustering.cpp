#include <trajectory_planner/clustering/obstacleClustering.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/utils.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout; using std::endl;
ros::Publisher currPoseVisPub;
ros::Publisher localCloudVisPub;
visualization_msgs::Marker currPoseMarker;
bool initCurrPose = false;

std::vector<Eigen::Vector3d> currCloud;

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

void publishLocalCloudVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (currCloud.size() != 0){
			pcl::PointXYZ pt;
			pcl::PointCloud<pcl::PointXYZ> cloud;

			for (int i=0; i<int(currCloud.size()); ++i){
				pt.x = currCloud[i](0);
				pt.y = currCloud[i](1);
				pt.z = currCloud[i](2);
				cloud.push_back(pt);
			}

			cloud.width = cloud.points.size();
			cloud.height = 1;
			cloud.is_dense = true;
			cloud.header.frame_id = "map";

			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			localCloudVisPub.publish(cloudMsg);		
		}
		r.sleep();
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "test_obstacle_clustering");
	ros::NodeHandle nh;

	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	currPoseVisPub = nh.advertise<visualization_msgs::Marker>("/current_pose", 1000);
	localCloudVisPub = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud", 1000);
	std::thread currPoseVisWorker = std::thread(publishCurrPoseVis);
	std::thread localCloudVisWorker = std::thread(publishLocalCloudVis);

	obstacleClustering oc;
	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Test Clustering Node]: Request No. " << countLoop+1 << endl;
		cout << "[Test Clustering Node]: Wait for current pose point..." << endl;
		std::vector<double> currPose;
		while (ros::ok()){
			if (newMsg){
				currPose = newPoint;
				newMsg = false;
				cout << "[Test Clustering Nodeg]: current pose point OK. (" << currPose[0] << " " << currPose[1] << " " << currPose[2] << " " << currPose[3] << ")" << endl;
				
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
				currPoseMarker.scale.z = 0.2;
				currPoseMarker.color.a = 0.7;
				currPoseMarker.color.r = 1.0;
				currPoseMarker.color.g = 0.5;
				currPoseMarker.color.b = 1.0;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}


		ros::Time localCloudStartTime = ros::Time::now();
		// get local point cloud
		Eigen::Vector3d mapMin, mapMax;
		map->getCurrMapRange(mapMin, mapMax);
		double offset = 0.0;
		double regionSizeX = 5.0;
		double regionSizeY = 2.0; // half
		double groundHeight = 0.3;
		double cloudRes = 0.2;
		double angle = currPose[3];
		Eigen::Vector3d pCurr (currPose[0], currPose[1], currPose[2]);
		Eigen::Vector3d faceDirection (cos(angle), sin(angle), 0);
		Eigen::Vector3d sideDirection (-sin(angle), cos(angle), 0); // positive (left side)
		Eigen::Vector3d pOrigin = pCurr - offset * faceDirection;

		// find four vextex of the bounding boxes
		Eigen::Vector3d p1, p2, p3, p4;
		p1 = pOrigin + regionSizeY * sideDirection;
		p2 = pOrigin - regionSizeY * sideDirection;
		p3 = p1 + (regionSizeX + offset) * faceDirection;
		p4 = p2 + (regionSizeX + offset) * faceDirection;

		double xStart = std::min({p1(0), p2(0), p3(0), p4(0)});
		double xEnd = std::max({p1(0), p2(0), p3(0), p4(0)});
		double yStart = std::min({p1(1), p2(1), p3(1), p4(1)});
		double yEnd = std::max({p1(1), p2(1), p3(1), p4(1)});


		currCloud.clear();
		for (double ix=xStart; ix<=xEnd; ix+=cloudRes){
			for (double iy=yStart; iy<=yEnd; iy+=cloudRes){
				for (double iz=groundHeight; iz<=mapMax(2); iz+=cloudRes){
					Eigen::Vector3d p (ix, iy, iz);
					if ((p - pOrigin).dot(faceDirection) >= 0){
						// cout << p.transpose() << endl;
						if (map->isInflatedOccupied(p)){
							currCloud.push_back(p);
						}
					}
				}
			}
		}
		ros::Time localCloudEndTime = ros::Time::now();
		cout << "[Test Clustering Node]: local cloud obtain time: " << (localCloudEndTime - localCloudStartTime).toSec() << "s." << endl;

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}

	ros::spin();
	return 0;
}