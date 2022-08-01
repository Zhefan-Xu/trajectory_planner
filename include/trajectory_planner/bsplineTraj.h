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

namespace trajPlanner{
	struct optData{
		Eigen::MatrixXd controlPoints_; // control points

	};



	class bsplineTraj{
	private:
		ros::NodeHandle nh_;

		// bspline
		trajPlanner::bspline bspline_; // this is used to evaluate bspline. not for optimization
		trajPlanner::optData optData_; // all optimization information including control points

	public:
		bsplineTraj();
		void initControlPoints(const nav_msgs::Path& path);
		void evaluateTraj();


		// helper function
		void pathMsgToBsplinePoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
	};
}

#endif
