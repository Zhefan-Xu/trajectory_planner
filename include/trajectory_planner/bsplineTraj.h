/*
	FILE: bsplineTraj.h
	----------------------------
	b spline trajectory solver header based on occupancy grid map
*/
#ifndef BSPLINETRAJ_H
#define BSPLINETRAJ_H
#include <ros/ros.h>
#include <trajectory_planner/bspline.h>
#include <trajectory_planner/path_search/astarOcc.h>
#include <map_manager/occupancyMap.h>
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
		ros::Publisher astarVisPub_;

		// bspline
		trajPlanner::bspline bspline_; // this is used to evaluate bspline. not for optimization
		trajPlanner::optData optData_; // all optimization information including control points
		double ts_;

		// occupancy grid map and path search
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<AStar> pathSearch_;

		std::vector<std::vector<Eigen::Vector3d>> astarPaths_;

		// flag
		bool init_ = false;

	public:
		bsplineTraj();
		void init(const ros::NodeHandle& nh);
		void initParam();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map); // update occuapncy grid map
		void updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition); // used to initialize control points
		
		void makePlan();
		void findCollisionSeg(const Eigen::MatrixXd& controlPoints, std::vector<std::pair<int, int>>& collisionSeg); // find collision segment of current control points




		// visualization
		void visCB(const ros::TimerEvent& );
		void publishControlPoints();
		void publishCurrTraj();
		void publishAstarPath();

		// helper function
		std::vector<Eigen::Vector3d> evalTraj();
		nav_msgs::Path evalTrajToMsg(); // evaluate current trajectory based on the control point
		void pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
		void eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path);
	};
}

#endif
