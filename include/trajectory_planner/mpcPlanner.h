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
USING_NAMESPACE_ACADO
// ACADOvariables acadoVariables;
// ACADOworkspace acadoWorkspace;

namespace trajPlanner{
	class mpcPlanner{
	private:
		ros::NodeHandle nh_;

	public:
		mpcPlanner(const ros::NodeHandle& nh);
		void makePlan();
	};
}
#endif