/*
	file: polyTrajOctomap.h
	-----------------------
	minimum snap polynomial trajectory planner based on ocromap
*/

#ifndef POLYTRAJPLANNER_H
#define POLYTRAJPLANNER_H
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <trajectory_planner/polyTrajSolver.h>
#include <thread>
#include <mutex>

namespace trajPlanner{
	class polyTrajOctomap{
	private:
		ros::NodeHandle nh_;
		ros::ServiceClient mapClient_; // call octomap server

		int degree_; // polynomial degree
		double veld_; // desired velocity
		int diffDegree_; // differential degree
		int continuityDegree_; // continuity degree
		double regularizationWeights_; // paramters regularization
		double mapRes_;
		int maxIter_;
		double initR_;
		double fs_; //factor of shrink

		polyTrajSolver* trajSolver_; // trajectory solver
		std::vector<pose> path_; // waypoint path
		std::vector<double> collisionBox_;
		octomap::OcTree* map_;

		// visualization:
		ros::Publisher trajVisPub_;
		ros::Publisher samplePointVisPub_;
		ros::Publisher waypointVisPub_;
		ros::Publisher corridorVisPub_;
		nav_msgs::Path trajVisMsg_;
		visualization_msgs::MarkerArray samplePointMsg_;
		visualization_msgs::MarkerArray waypointMsg_;
		visualization_msgs::MarkerArray corridorMsg_;
		
	public:
		std::thread trajVisWorker_;
		std::thread samplePointVisWorker_;
		std::thread waypointVisWorker_;
		std::thread corridorVisWorker_;

		polyTrajOctomap();

		polyTrajOctomap(const ros::NodeHandle& nh, std::vector<double> collisionBox, int degree, int diffDegree, int continuityDegree, double veld, double regularizationWeights, double mapRes, int maxIter, double initR, double fs);
		
		// update octomap
		void updateMap();

		// initialize solver
		polyTrajSolver*  initSolver();
		void freeSolver();

		// update waypoint path
		void updatePath(const std::vector<pose>& path);
		void insertWaypoint(const std::set<int>& seg);
		void adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& collisionVec);

		// CORE FUNCTION:
		void makePlan(std::vector<pose>& trajectory, double delT=0.1);
		void makePlanAddingWaypoint(std::vector<pose>& trajectory, double delT); // adding point for collision avoidance
		void makePlanCorridorConstraint(std::vector<pose>& trajectory, double delT); // adding corridor constraint for collision avoidance

		// collision checking
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=true);
		bool checkCollisionPoint(const pose& pTraj);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);
		bool checkCollisionLine(const pose& pTraj1, const pose& pTraj2);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, std::vector<int>& collisionIdx);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg);


		double getDegree();
		double getVelD();
		double getDiffDegree();
		double getInitialRadius();
		double getShrinkFactor();

		// visualizaton:
		void updateTrajVisMsg(const std::vector<pose>& trajectory);
		void updateCorridorVisMsg(const std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, const std::vector<double>& corridorSizeVec);
		void publishTrajectory();
		void publishSamplePoint();
		void publishWaypoint();
		void publishCorridor();

		// conversion helper:
		void pose2Octomap(const pose& pTraj, octomap::point3d& p);
	};

	std::ostream &operator<<(std::ostream &os, polyTrajOctomap& polyPlanner){
        os << "========================INFO========================\n";
        os << "[Trajectory Planner INFO]: minimum snap polynomial trajectory planner with octomap\n";
        os << "[Polynomial Degree]: " << polyPlanner.getDegree() << "\n";
        os << "[Desired Velocity]: " << polyPlanner.getVelD() << "\n";
        os << "[Differential Degree]: " << polyPlanner.getDiffDegree() << "\n";
        os << "[Initial Radius]: " << polyPlanner.getInitialRadius() << "\n";
        os << "[Shrink Factor]: " << polyPlanner.getShrinkFactor() << "\n";
        os << "====================================================";
        return os;
    }
}

#endif