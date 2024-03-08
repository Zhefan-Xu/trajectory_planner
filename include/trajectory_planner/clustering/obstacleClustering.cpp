/*
	FILE: obstacleClustering.cpp
	------------------------
	function implementation of static obstacle clustering
*/

#include <trajectory_planner/clustering/obstacleClustering.h>

obstacleClustering::obstacleClustering(){

}



void obstacleClustering::getStaticObstacles(const std::vector<Eigen::Vector3d>& localCloud){
	// run DBSCAN clustering
	this->runDBSCAN(localCloud, this->initialClusters_);

}

void obstacleClustering::runDBSCAN(const std::vector<Eigen::Vector3d>& localCloud, std::vector<pointCluster>& cloudClusters){
	std::vector<Point> points;
	for (int i=0; i<int(localCloud.size()); ++i){
		points.push_back(Point (localCloud[i](0), localCloud[i](1), localCloud[i](2)));
	}

	double eps = 0.5;
	double minPts = 15;
	this->dbscan_.reset(new DBSCAN (eps, minPts, points));
	this->dbscan_->run();


	std::vector<vector<int>> cluster = this->dbscan_->getCluster();
	cloudClusters.clear();
	for (int i=0; i<int(cluster.size()); ++i){ // number of cluster
		Eigen::Vector3d centroid (0.0, 0.0, 0.0);
		Eigen::Vector3d clusterMin = localCloud[cluster[i][0]];
		Eigen::Vector3d clusterMax = localCloud[cluster[i][0]];
		std::vector<Eigen::Vector3d> cloudCluster;
		for (int j=0; j<int(cluster[i].size()); ++j){
			Eigen::Vector3d p = localCloud[cluster[i][j]];
			cloudCluster.push_back(p);
			centroid += p/int(cluster[i].size());

			if (p(0) < clusterMin(0)){
				clusterMin(0) = p(0);
			}
			else if (p(0) > clusterMax(0)){
				clusterMax(0) = p(0);
			}

			if (p(1) < clusterMin(1)){
				clusterMin(1) = p(1);
			}
			else if (p(1) > clusterMax(1)){
				clusterMax(1) = p(1);
			}

			if (p(2) < clusterMin(2)){
				clusterMin(2) = p(2);
			}
			else if (p(2) > clusterMax(2)){
				clusterMax(2) = p(2);
			}
		}
		cloudClusters.push_back(pointCluster (cloudCluster, i, centroid, clusterMin, clusterMax));
	}
}


std::vector<pointCluster> obstacleClustering::getInitialCluster(){
	return this->initialClusters_;
}