/*
	FILE: obstacleClustering.h
	------------------------
	function header of static obstacle clustering
*/
#ifndef OBSTALCE_CLUSTERING_H
#define OBSTALCE_CLUSTERING_H

#include <trajectory_planner/clustering/DBSCAN.h>
#include <thread>
#include <Eigen/Eigen>


struct pointCluster{
	std::vector<Eigen::Vector3d> points;
	int id;
	Eigen::Vector3d centroid;
	Eigen::Vector3d clusterMin, clusterMax;

	pointCluster(const std::vector<Eigen::Vector3d>& _points, int _id, const Eigen::Vector3d& _centroid, 
		         const Eigen::Vector3d& _clusterMin, const Eigen::Vector3d& _clusterMax){
		points = _points;
		id = _id;
		centroid = _centroid;
		clusterMin = _clusterMin;
		clusterMax = _clusterMax;
	}
};


class obstacleClustering{
	private:
		std::shared_ptr<DBSCAN> dbscan_;
		std::vector<pointCluster> initialClusters_;

	public:
		obstacleClustering();
		void getStaticObstacles(const std::vector<Eigen::Vector3d>& localCloud);

		void runDBSCAN(const std::vector<Eigen::Vector3d>& localCloud, std::vector<pointCluster>& cloudClusters);

		std::vector<pointCluster> getInitialCluster();
};


#endif