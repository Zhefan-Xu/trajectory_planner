/*
	FILE: polyTrajOccMap.h
	-------------------------
	minimun snap polynomial trajectory planner based on occupancy grid map
*/

#ifndef POLYTRAJPLANNEROCCMAP_H
#define POLYTRAJPLANNEROCCMAP_H
#include <ros/ros.h>
#include <trajectory_planner/utils.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/polyTrajSolver.h>
#include <nav_msgs/Path.h>

namespace trajPlanner{
	class polyTrajOccMap{
	private:
		ros::NodeHandle nh_;
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<trajPlanner::polyTrajSolver> trajSolver_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTrajSolver_;
		std::vector<pose> path_;
		geometry_msgs::Twist initVel_;
		geometry_msgs::Twist initAcc_;

		// parameters
		int polyDegree_;
		int diffDegree_;
		int continuityDegree_;
		double desiredVel_;
		double initR_; // initial corridor constraint radius
		double timeout_;

		// status
		bool findValidTraj_ = false;

	public:
		polyTrajOccMap(const ros::NodeHandle& nh);
		void initParam();
		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void initSolver();
		void initPWLSolver();

		// update waypoint path
		void updatePath(const nav_msgs::Path& path);
		void updatePath(const std::vector<pose>& path);

		// initial condition
		void updateInitVel(double vx, double vy, double vz);
		void updateInitVel(const geometry_msgs::Twist& v);
		void updateInitAcc(double ax, double ay, double az);
		void updateInitAcc(const geometry_msgs::Twist& a);
		void setDefaultInit();

		void makePlan();
	};
}

#endif