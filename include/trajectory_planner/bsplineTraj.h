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
#include <trajectory_planner/solver/lbfgs.hpp>
#include <map_manager/occupancyMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

const int bsplineDegree = 3;
namespace trajPlanner{
	struct optData{
		Eigen::MatrixXd controlPoints; // control points
		std::vector<std::vector<Eigen::Vector3d>> guidePoints; // p
		std::vector<std::vector<Eigen::Vector3d>> guideDirections; // v
		std::vector<bool> findGuidePoint;
	};



	class bsplineTraj{
	private:
		ros::NodeHandle nh_;
		ros::Timer visTimer_;
		ros::Publisher controlPointsVisPub_;
		ros::Publisher currTrajVisPub_;
		ros::Publisher astarVisPub_;
		ros::Publisher guidePointsVisPub_;


		// bspline
		trajPlanner::bspline bspline_; // this is used to evaluate bspline. not for optimization
		trajPlanner::optData optData_; // all optimization information including control points
		double ts_;
		double dthresh_;
		double maxVel_;
		double maxAcc_;
		double weightDistance_;
		double weightSmoothness_;
		double weightFeasibility_;

		// occupancy grid map and path search
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<AStar> pathSearch_;

		std::vector<std::pair<int, int>> collisionSeg_;
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
		void findCollisionSeg(const Eigen::MatrixXd& controlPoints); // find collision segment of current control points
		void pathSearch();
		void assignPVpairs();
		void optimize();

		// cost functions
		double solverCostFunction(void* func_data, const double* x, double* grad, const int n); // deal with solver
		double costFunction(const double* x, double* grad, const int n);
		void getDistanceCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // collision
		void getSmoothnessCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // trajectory
		void getFeasibilityCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // velocity and acceleration


		// visualization
		void visCB(const ros::TimerEvent& );
		void publishControlPoints();
		void publishCurrTraj();
		void publishAstarPath();
		void publishGuidePoints();

		// helper functionin
		std::vector<Eigen::Vector3d> evalTraj();
		nav_msgs::Path evalTrajToMsg(); // evaluate current trajectory based on the control point
		void pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
		void eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path);

		// inline function
		inline bool findGuidePointFromPath(const Eigen::Vector3d& controlPoint, const Vector3d& tangentDirection, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint);
		inline bool adjustGuidePoint(const Eigen::Vector3d& controlPoint, Eigen::Vector3d& guidePoint);
	};
	inline bool bsplineTraj::findGuidePointFromPath(const Eigen::Vector3d& controlPoint, const Vector3d& tangentDirection, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint){
		size_t initIdx = int(path.size()/2); // start from the middle point of the path
		Eigen::Vector3d pathPointDirection = path[initIdx] - controlPoint;
		double dotProduct = tangentDirection.dot(pathPointDirection);
		int searchDirection = 1;
		if (dotProduct > 0){ // need to backward search
			searchDirection = -1;
		}
		else if (dotProduct < 0){ // need to do forward search
			searchDirection = 1;
		}
		else{ // happen to find our guide point
			guidePoint = path[initIdx];
			return true;
		}

		// TODO: CORNER CASE: ONLY ONE SEGMENT IS NOT COLLISION FREE!!!!!!!!!!!!!!!!

		// start search for the best guide point
		double prevDotProduct = dotProduct;
		for (size_t i=initIdx+searchDirection; i>=0 and i<path.size(); i+=searchDirection){
			pathPointDirection = path[i] - controlPoint;
			dotProduct = tangentDirection.dot(pathPointDirection); 
			if (dotProduct == 0){ // find the exact control point
				guidePoint = path[i];
				return true;
			}
			else if (dotProduct * prevDotProduct < 0){
				// interpolate those two points in path
				Eigen::Vector3d currPoint = path[i];
				Eigen::Vector3d prevPoint = path[i-searchDirection];
				double currLength = std::abs(dotProduct);
				double totalLength = std::abs(dotProduct) + std::abs(prevDotProduct);
				Eigen::Vector3d interPoint = currPoint + (prevPoint - currPoint) * (currLength / totalLength);\
				guidePoint = interPoint;
				bool successAdjustment = this->adjustGuidePoint(controlPoint, guidePoint);
				return successAdjustment;

			}
			prevDotProduct = dotProduct;
		}
		return false;
	}

	inline bool bsplineTraj::adjustGuidePoint(const Eigen::Vector3d& controlPoint, Eigen::Vector3d& guidePoint){
		double length = (guidePoint - controlPoint).norm();
		if (length > 1e-5){
			bool hasCollision = false;
			for (double a=length; a>=0.0; a-=this->map_->getRes()){
				Eigen::Vector3d interpolatePoint = (a/length) * guidePoint + (1-a/length) * controlPoint;
				hasCollision = this->map_->isInflatedOccupied(interpolatePoint);
				if (hasCollision){
					a += this->map_->getRes();
					guidePoint = (a/length) * guidePoint + (1-a/length) * controlPoint;
					return true;
				}
			}
		}
		else{
			return false;
		}
		return false;
	}

}

#endif
