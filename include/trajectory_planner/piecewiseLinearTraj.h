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
		double desiredVel_ = 1.0;
		double desiredAngularVel_ = 0.5;
		std::vector<trajPlanner::pose> path_;
		std::vector<double> desiredTime_;


	public:
		pwlTraj(const ros::NodeHandle& nh);
		void updatePath(const nav_msgs::Path& path, bool useYaw=false);
		void updatePath(const std::vector<trajPlanner::pose>& path, bool useYaw=false);
		void updatePath(const nav_msgs::Path& path, double desiredVel, bool useYaw=false);
		void updatePath(const std::vector<trajPlanner::pose>& path, double desiredVel, bool useYaw=false);	
		void avgTimeAllocation(bool useYaw=false);
		void avgTimeAllocation(double desiredVel, bool useYaw=false);
		void adjustHeading(const geometry_msgs::Quaternion& quat);
		void adjustHeading(double yaw);

		void makePlan(nav_msgs::Path& trajectory, double delT);
		void makePlan(std::vector<trajPlanner::pose>& trajectory, double delT);

		geometry_msgs::PoseStamped getPose(double t);
		std::vector<double> getTimeKnot();
		double getDuration();
		double getDesiredVel();
		double getDesiredAngularVel();
		geometry_msgs::PoseStamped getFirstPose();
	};
}


#endif