/*
	FILE: mpcPlanner.h
	----------------------------
	mpc trajectory solver header based on occupancy grid map
*/

#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H
#include <ros/ros.h>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
USING_NAMESPACE_ACADO
// ACADOvariables acadoVariables;
// ACADOworkspace acadoWorkspace;
using std::cout; using std::endl;
namespace trajPlanner{
	class mpcPlanner{
	private:
		std::string ns_;
		std::string hint_;
		ros::NodeHandle nh_;


		double ts_; // timestep
		Eigen::Vector3d currPos_;
		std::vector<Eigen::Vector3d> inputTraj_;

		// parameters
		int horizon_;

	public:
		mpcPlanner(const ros::NodeHandle& nh);
		void initParam();


		void updateCurrPosition(const Eigen::Vector3d& pos);
		void updatePath(const nav_msgs::Path& path, double ts);
		void updatePath(const std::vector<Eigen::Vector3d>& path, double ts);
		void makePlan();

		void getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj);

	};
}
#endif