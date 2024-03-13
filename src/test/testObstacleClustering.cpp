#include <trajectory_planner/clustering/obstacleClustering.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/utils.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout; using std::endl;
ros::Publisher currPoseVisPub;
ros::Publisher localCloudVisPub;
ros::Publisher initialClusterBBoxPub;
ros::Publisher rotatedBBoxPub;
ros::Publisher refinedBBoxPub;
visualization_msgs::Marker currPoseMarker;
bool initCurrPose = false;

std::vector<Eigen::Vector3d> currCloud;
std::vector<pointCluster> initialClusters;
std::vector<bboxVertex> rotatedBBoxVertices;
std::vector<bboxVertex> refinedBBoxVertices;
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

visualization_msgs::MarkerArray cluster2MarkerArray(const std::vector<pointCluster>& cluster, double r, double g, double b){
    visualization_msgs::Marker line;
    visualization_msgs::MarkerArray lines;
    line.header.frame_id = "map";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.ns = "initial box";  
    line.scale.x = 0.06;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(0.15);
	for (int i=0; i<int(cluster.size()); ++i){
		double x = cluster[i].centroid(0); 
        double y = cluster[i].centroid(1); 
        double z = cluster[i].centroid(2); 

        double x_width = cluster[i].clusterMax(0) - cluster[i].clusterMin(0);
        double y_width = cluster[i].clusterMax(1) - cluster[i].clusterMin(1);
        double z_width = cluster[i].clusterMax(2) - cluster[i].clusterMin(2);

        vector<geometry_msgs::Point> verts;
        geometry_msgs::Point p;
        // vertice 0
        p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 1
        p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 2
        p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 3
        p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 4
        p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 5
        p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 6
        p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 7
        p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);
        
        int vert_idx[12][2] = {
            {0,1},
            {1,2},
            {2,3},
            {0,3},
            {0,4},
            {1,5},
            {3,7},
            {2,6},
            {4,5},
            {5,6},
            {4,7},
            {6,7}
        };
        
        for (size_t i=0;i<12;i++){
            line.points.push_back(verts[vert_idx[i][0]]);
            line.points.push_back(verts[vert_idx[i][1]]);
        }
        
        lines.markers.push_back(line);
        
        line.id++;
    }	
    return lines;
}

visualization_msgs::MarkerArray vertex2MarkerArray(const std::vector<bboxVertex>& bboxVertices, double r, double g, double b, std::string ns){
    visualization_msgs::Marker line;
    visualization_msgs::MarkerArray lines;

    line.header.frame_id = "map";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.ns = ns;  
    line.scale.x = 0.06;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(0.15);
    Eigen::Vector3d vertex_pose;
    for(int i=0; i<int(bboxVertices.size()); ++i){
        bboxVertex v = bboxVertices[i];
        std::vector<geometry_msgs::Point> verts;
        geometry_msgs::Point p;

		for (int j=0; j<int(v.vert.size());++j){
			p.x = v.vert[j](0); p.y = v.vert[j](1); p.z = v.vert[j](2);
        	verts.push_back(p);
		}

        int vert_idx[12][2] = {
            {0,1},
            {1,2},
            {2,3},
            {0,3},
            {0,4},
            {1,5},
            {3,7},
            {2,6},
            {4,5},
            {5,6},
            {4,7},
            {6,7}
        };
        for (int j=0;j<12;++j){
            line.points.push_back(verts[vert_idx[j][0]]);
            line.points.push_back(verts[vert_idx[j][1]]);
        }
        lines.markers.push_back(line);
        line.id++;
    }
	return lines;	
}

void publishInitialClusterBBoxVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (initialClusters.size() != 0){
			visualization_msgs::MarkerArray bboxVis = cluster2MarkerArray(initialClusters, 1, 0, 0);
			initialClusterBBoxPub.publish(bboxVis);
		}
		r.sleep();
	}
}

void publishRotatedBBoxVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (rotatedBBoxVertices.size() != 0){
			visualization_msgs::MarkerArray bboxVis = vertex2MarkerArray(rotatedBBoxVertices, 0, 0, 1, "initial_rotated");
			rotatedBBoxPub.publish(bboxVis);
		}
		r.sleep();
	}
}

void publishRefinedBBoxVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (refinedBBoxVertices.size() != 0){
			visualization_msgs::MarkerArray bboxVis = vertex2MarkerArray(refinedBBoxVertices, 0, 1, 1, "refined_rotated");
			refinedBBoxPub.publish(bboxVis);
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
	initialClusterBBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/initial_cluster_bbox", 1000);
	rotatedBBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/rotated_initial_bbox", 1000);
	refinedBBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/refined_bbox", 1000);
	std::thread currPoseVisWorker = std::thread(publishCurrPoseVis);
	std::thread localCloudVisWorker = std::thread(publishLocalCloudVis);
	std::thread initialClusterBBoxVisWorker = std::thread(publishInitialClusterBBoxVis);
	std::thread rotatedBBoxVisWorker = std::thread(publishRotatedBBoxVis);
	std::thread refinedBBoxVisWorker = std::thread(publishRefinedBBoxVis);


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
				cout << "[Test Clustering Node]: current pose point OK. (" << currPose[0] << " " << currPose[1] << " " << currPose[2] << " " << currPose[3] << ")" << endl;
				
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


		
		// get local point cloud
		ros::Time localCloudStartTime = ros::Time::now();
		Eigen::Vector3d mapMin, mapMax;
		map->getCurrMapRange(mapMin, mapMax);
		double offset = 0.0;
		double regionSizeX = 5.0;
		double regionSizeY = 2.0; // half
		double groundHeight = 0.4;
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
						if (map->isInMap(p) and map->isInflatedOccupied(p)){
							currCloud.push_back(p);
						}
					}
				}
			}
		}
		ros::Time localCloudEndTime = ros::Time::now();
		cout << "[Test Clustering Node]: local cloud obtain time: " << (localCloudEndTime - localCloudStartTime).toSec() << "s." << endl;


		obstacleClustering oc (cloudRes);
		// cluster the local pointcloud into bounding boxes
		ros::Time clusterStartTime = ros::Time::now();
		oc.run(currCloud);
		initialClusters = oc.getInitialCluster();
		rotatedBBoxVertices = oc.getRotatedInitialBBoxes();
		refinedBBoxVertices = oc.getRefinedBBoxes();
		ros::Time clusterEndTime = ros::Time::now();
		cout << "[Test Clustering Node]: size of objects: " << refinedBBoxVertices.size() << endl;
		cout << "[Test Clustering Node]: cluster time: " << (clusterEndTime - clusterStartTime).toSec() << "s." << endl;

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}

	ros::spin();
	return 0;
}