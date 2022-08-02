/*
	FILE: bsplineTraj.h
	----------------------------
	b spline trajectory solver header based on occupancy grid map
*/
#ifndef BSPLINETRAJ_H
#define BSPLINETRAJ_H
#include <ros/ros.h>
#include <trajectory_planner/bspline.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

const int bsplineDegree = 3;
namespace trajPlanner{
	struct optData{
		Eigen::MatrixXd controlPoints; // control points

	};



	class bsplineTraj{
	private:
		ros::NodeHandle nh_;
		ros::Timer visTimer_;
		ros::Publisher controlPointsVisPub_;
		ros::Publisher currTrajVisPub_;

		// bspline
		trajPlanner::bspline bspline_; // this is used to evaluate bspline. not for optimization
		trajPlanner::optData optData_; // all optimization information including control points
		double ts_;

		// flag
		bool init_ = false;

	public:
		bsplineTraj();
		void init(const ros::NodeHandle& nh); 
		void initParam();
		void registerPub();
		void registerCallback();
		void updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition); // used to initialize control points
		std::vector<Eigen::Vector3d> evalTraj();
		nav_msgs::Path evalTrajToMsg(); // evaluate current trajectory based on the control point


		// visualization
		void visCB(const ros::TimerEvent& );
		void publishControlPoints();
		void publishCurrTraj();

		// helper function
		void pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
		void eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path);
	};
}

#endif
