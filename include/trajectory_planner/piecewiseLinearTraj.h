/*
	File: piecewiseLinearTraj.h
	----------------------------
	generate piecewise-linear trajectory from waypoint path
*/

#ifndef PIECEWISELINEARTRAJ_H
#define PIECEWISELINEARTRAJ_H

#include <ros/ros.h>
#include <trajectory_planner/utils.h>
#include <nav_msgs/Path.h>

using std::cout; using std::endl;

namespace trajPlanner{
	class pwlTraj{
	private:
		ros::NodeHandle nh_;
		double desiredVel_;
		double desiredAngularVel_;
		std::vector<trajPlanner::pose> path_;
		std::vector<double> desiredTime_;


	public:
		pwlTraj(const ros::NodeHandle& nh);
		void updatePath(const nav_msgs::Path& path);
		void updatePath(const std::vector<trajPlanner::pose>& path);
		void avgTimeAllocation();
		void makePlan();

		const geometry_msgs::PoseStamped& getPose(double t);
	};
}


#endif