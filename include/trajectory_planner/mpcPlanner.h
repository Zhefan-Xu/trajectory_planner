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
#include <visualization_msgs/MarkerArray.h>
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
		ros::Publisher mpcTrajVisPub_;

		ros::Timer visTimer_;

		double ts_; // timestep
		Eigen::Vector3d currPos_;
		Eigen::Vector3d currVel_;
		std::vector<Eigen::Vector3d> inputTraj_;
		bool firstTime_ = true;
		VariablesGrid currentStatesSol_;
		VariablesGrid currentControlsSol_;
		std::vector<Eigen::Vector3d> currentTraj_;



		// parameters
		int horizon_;
		double maxVel_ = 1.0;
		double maxAcc_ = 1.0;
		double constraintSlackRatio_;
		double zRangeMin_;
		double zRangeMax_;

	public:
		mpcPlanner(const ros::NodeHandle& nh);
		void initParam();
		void registerPub();
		void registerCallback();

		void updateMaxVel(double maxVel);
		void updateMaxAcc(double maxAcc);
		void updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
		void updatePath(const nav_msgs::Path& path, double ts);
		void updatePath(const std::vector<Eigen::Vector3d>& path, double ts);
		void makePlan();

		void getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj);
		VariablesGrid getReferenceTraj();

		void getTrajectory(std::vector<Eigen::Vector3d>& traj);
		void getTrajectory(nav_msgs::Path& traj);

		Eigen::Vector3d getPos(double t);
		Eigen::Vector3d getVel(double t);
		Eigen::Vector3d getAcc(double t);

		void visCB(const ros::TimerEvent&);
		void publishMPCTrajectory();


	};
}
#endif