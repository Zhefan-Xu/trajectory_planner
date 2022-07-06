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
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <thread>
#include <mutex>

namespace trajPlanner{
	class polyTrajOctomap{
	private:
		ros::NodeHandle nh_;
		ros::ServiceClient mapClient_; // call octomap server

		int polyDegree_; // polynomial degree
		double desiredVel_; // desired velocity
		int diffDegree_; // differential degree
		int continuityDegree_; // continuity degree
		double regularizationWeights_; // paramters regularization
		double mapRes_;
		int maxIter_;
		double timeout_;
		bool mode_; // collision avoidance mode. (True: adding waypoint, False: corridor constraint) 
		bool softConstraint_;
		double softConstraintRadius_;
		double delT_;
		double initR_;
		double fs_; //factor of shrink
		double corridorRes_;
		bool findValidTraj_; // find valid solution

		// initial condition
		geometry_msgs::Twist initVel_;
		geometry_msgs::Twist initAcc_;

		polyTrajSolver* trajSolver_; // trajectory solver
		pwlTraj* pwlTrajSolver_; // piecewise linear trajectory solver
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

		polyTrajOctomap(const ros::NodeHandle& nh);
		
		// update octomap
		void updateMap();

		// initialize solver
		polyTrajSolver*  initSolver();
		void freeSolver();
		pwlTraj* initPWLSolver();
		void freePWLSolver();

		// update waypoint path
		void updatePath(const nav_msgs::Path& path);
		void updatePath(const std::vector<pose>& path);
		void insertWaypoint(const std::set<int>& seg);
		void adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& collisionVec);


		// Initial condition
		void updateInitVel(double vx, double vy, double vz);
		void updateInitVel(const geometry_msgs::Twist& v);
		void updateInitAcc(double ax, double ay, double az);
		void updateInitAcc(const geometry_msgs::Twist& a);
		void setDefaultInit();

		// CORE FUNCTION:
		void makePlan(); // no return. Get trajectory by this object
		void makePlan(nav_msgs::Path& trajectory, double delT=0.1);
		void makePlan(std::vector<pose>& trajectory, double delT=0.1);
		void makePlanAddingWaypoint();
		void makePlanAddingWaypoint(std::vector<pose>& trajectory, double delT); // adding point for collision avoidance
		void makePlanCorridorConstraint();
		void makePlanCorridorConstraint(std::vector<pose>& trajectory, double delT); // adding corridor constraint for collision avoidance

		// collision checking
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=true);
		bool checkCollisionPoint(const pose& pTraj);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);
		bool checkCollisionLine(const pose& pTraj1, const pose& pTraj2);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, std::vector<int>& collisionIdx);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg);

		// get poses at specific time:
		geometry_msgs::PoseStamped getPose(double t); 
		double getDuration();

		double getDegree();
		double getDiffDegree();
		double getContinuityDegree();
		double getDesiredVel();
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
		void trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::Path& trajectory);
	};

	std::ostream &operator<<(std::ostream &os, polyTrajOctomap& polyPlanner){
        os << "========================INFO========================\n";
        os << "[Trajectory Planner INFO]: minimum snap polynomial trajectory planner with octomap\n";
        os << "[Polynomial Degree]: " << polyPlanner.getDegree() << "\n";
        os << "[Differential Degree]: " << polyPlanner.getDiffDegree() << "\n";
        os << "[Continuity Degree]: " << polyPlanner.getContinuityDegree() << "\n";
        os << "[Desired Velocity]: " << polyPlanner.getDesiredVel() << "\n";
        os << "====================================================";
        return os;
    }
}

#endif