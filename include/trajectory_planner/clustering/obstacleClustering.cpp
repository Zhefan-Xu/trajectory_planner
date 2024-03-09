/*
	FILE: obstacleClustering.cpp
	------------------------
	function implementation of static obstacle clustering
*/

#include <trajectory_planner/clustering/obstacleClustering.h>

obstacleClustering::obstacleClustering(double res){
	this->res_ = res;
}


void obstacleClustering::getStaticObstacles(const std::vector<Eigen::Vector3d>& localCloud){
	// run DBSCAN clustering
	this->runDBSCAN(localCloud, this->initialClusters_);

	// run KMeans clustering for each DBSCAN cluster
	std::vector<pointCluster> refinedCluster = this->initialClusters_;
	std::vector<bool> refineComplete (int(refinedCluster.size()), false);
	std::vector<bboxVertex> rotatedInitialBBoxes;
	for (int level=0; level<this->treeLevel_; ++level){
		for (int n=0; n<int(refinedCluster.size()); ++n){
			if (not refineComplete[n]){
				// if the refinement is not completed, find the bbox orientation and density
				bboxVertex vertex;
				double density = this->getOrientation(refinedCluster[n], vertex);
				rotatedInitialBBoxes.push_back(vertex);
				// if the density is greater than threshold, mark it as complete

				// else split it into two
			}
		}
	}
	this->rotatedInitialBBoxes_ = rotatedInitialBBoxes;
}

void obstacleClustering::runDBSCAN(const std::vector<Eigen::Vector3d>& localCloud, std::vector<pointCluster>& cloudClusters){
	std::vector<Point> points;
	for (int i=0; i<int(localCloud.size()); ++i){
		points.push_back(Point (localCloud[i](0), localCloud[i](1), localCloud[i](2)));
	}

	this->dbscan_.reset(new DBSCAN (this->eps_, this->minPts_, points));
	this->dbscan_->run();


	std::vector<vector<int>> cluster = this->dbscan_->getCluster();
	cloudClusters.clear();
	for (int i=0; i<int(cluster.size()); ++i){ // number of cluster
		Eigen::Vector3d clusterMin = localCloud[cluster[i][0]];
		Eigen::Vector3d clusterMax = localCloud[cluster[i][0]];
		std::vector<Eigen::Vector3d> cloudCluster;
		for (int j=0; j<int(cluster[i].size()); ++j){
			Eigen::Vector3d p = localCloud[cluster[i][j]];
			cloudCluster.push_back(p);

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
		Eigen::Vector3d centroid = (clusterMin + clusterMax)/2.0;
		cloudClusters.push_back(pointCluster (cloudCluster, i, centroid, clusterMin, clusterMax));
	}
}


double obstacleClustering::getOrientation(const pointCluster& cluster, bboxVertex& vertex){
	double maxDensity = 0.0;
	double bestAngle = 0.0;
	bboxVertex bestVertex;

	for (int i=0; i<this->angleDiscreteNum_; ++i){
		double angle = M_PI * double(i)/double(this->angleDiscreteNum_);
		Eigen::Matrix3d rot;
		rot << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0,  0, 0, 1;
		Eigen::Vector3d rotClusterMin = cluster.points[0];
		Eigen::Vector3d rotClusterMax = cluster.points[0];		
		for (int n=0; n<int(cluster.points.size()); ++n){
			Eigen::Vector3d p = cluster.points[n];
			Eigen::Vector3d pNew = rot * (p - cluster.centroid) + cluster.centroid;
			
			if (pNew(0) < rotClusterMin(0)){
				rotClusterMin(0) = pNew(0);
			}
			else if (pNew(0) > rotClusterMax(0)){
				rotClusterMax(0) = pNew(0);
			}

			if (pNew(1) < rotClusterMin(1)){
				rotClusterMin(1) = pNew(1);
			}
			else if (pNew(1) > rotClusterMax(1)){
				rotClusterMax(1) = pNew(1);
			}

			if (pNew(2) < rotClusterMin(2)){
				rotClusterMin(2) = pNew(2);
			}
			else if (pNew(2) > rotClusterMax(2)){
				rotClusterMax(2) = pNew(2);
			}		
		}

		bboxVertex currVertex (rotClusterMin, rotClusterMax, -angle);
		double lengthNum = (rotClusterMax(0) - rotClusterMin(0))/this->res_;
		double widthNum = (rotClusterMax(1) - rotClusterMin(1))/this->res_;
		double heightNum = (rotClusterMax(2) - rotClusterMin(2))/this->res_;
		double density = int(cluster.points.size())/(lengthNum * widthNum * heightNum);
		if (density > maxDensity){
			maxDensity = density;
			bestAngle = angle;
			bestVertex = currVertex;
		}
	}

	// rotate the vertex 
	for (int i=0; i<int(bestVertex.vert.size()); ++i){
		Eigen::Matrix3d rot;
		rot << cos(-bestAngle), -sin(-bestAngle), 0, sin(-bestAngle), cos(-bestAngle), 0, 0, 0, 1;
		Eigen::Vector3d v = bestVertex.vert[i];
		Eigen::Vector3d newVertex = rot * (v - cluster.centroid) + cluster.centroid;
		bestVertex.vert[i] = newVertex;
	}
	vertex = bestVertex;
	return maxDensity;
}


std::vector<pointCluster> obstacleClustering::getInitialCluster(){
	return this->initialClusters_;
}

std::vector<bboxVertex> obstacleClustering::getRotatedInitialBBoxes(){
	return this->rotatedInitialBBoxes_;
}