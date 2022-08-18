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
#include <trajectory_planner/utils.h>
#include <map_manager/occupancyMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

const int bsplineDegree = 3;
namespace trajPlanner{
	struct optData{
		Eigen::MatrixXd controlPoints; // control points
		std::vector<std::vector<Eigen::Vector3d>> guidePoints; // p
		std::vector<std::vector<Eigen::Vector3d>> guideDirections; // v
		std::vector<bool> findGuidePoint;
		std::vector<Eigen::Vector3d> dynamicObstaclesPos;
		std::vector<Eigen::Vector3d> dynamicObstaclesVel;
		std::vector<Eigen::Vector3d> dynamicObstaclesSize;
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
		double weightDynamicObstacle_;
		double notCheckRatio_ = 0.0;
		double minHeight_;
		double maxHeight_;
		double uncertainAwareFactor_;
		double predHorizon_; // prediction horizon for dynamic obstacles in seconds
		double distThreshDynamic_; // distance threshold to keep with dynamic obstacles

		// occupancy grid map and path search
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<AStar> pathSearch_;

		std::vector<std::pair<int, int>> collisionSeg_;
		std::vector<std::vector<Eigen::Vector3d>> astarPaths_;
		std::vector<std::vector<Eigen::Vector3d>> astarPathsSC_; // shortcut paths

		// flag
		bool init_ = false;

	public:
		// std::thread visualizationWorker_;

		bsplineTraj();
		bsplineTraj(const ros::NodeHandle& nh);
		void init(const ros::NodeHandle& nh);
		void initParam();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map); // update occuapncy grid map
		bool updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition); // used to initialize control points
		void updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize); // position, velocity, size

		bool makePlan();
		bool makePlan(nav_msgs::Path& trajectory);
		void clear();
		void findCollisionSeg(const Eigen::MatrixXd& controlPoints, std::vector<std::pair<int, int>>& collisionSeg); // find collision segment of current control points
		bool pathSearch(const std::vector<std::pair<int, int>>& collisionSeg, std::vector<std::vector<Eigen::Vector3d>>& paths);
		void findGuidePointSemiCircle(int controlPointIdx, const std::pair<int, int>& seg, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint);
		bool isReguideRequired(std::vector<std::pair<int, int>>& reguideCollisionSeg);
		bool optimizeTrajectory();
		int optimize(); // optimize once
		void adjustTrajFeasibility(nav_msgs::Path& traj);

		// cost functions
		static double solverCostFunction(void* func_data, const double* x, double* grad, const int n); // deal with solver
		double costFunction(const double* x, double* grad, const int n);
		void getDistanceCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // collision
		void getSmoothnessCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // trajectory
		void getFeasibilityCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // velocity and acceleration
		void getDynamicObstacleCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // dynamic obstacle

		// visualization
		// void startVisualization();
		void visCB(const ros::TimerEvent&);
		void publishControlPoints();
		void publishCurrTraj();
		void publishAstarPath();
		void publishGuidePoints();

		// user functions
		geometry_msgs::PoseStamped getPose(double t);
		double getDuration();
		bool isCurrTrajValid();
		bool isCurrTrajValid(Eigen::Vector3d& firstCollisionPos);

		// helper functionin
		std::vector<Eigen::Vector3d> evalTraj();
		nav_msgs::Path evalTrajToMsg(); // evaluate current trajectory based on the control point
		void pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
		void eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path);

		// inline function
		inline bool isGoalValid();
		inline bool checkCollisionLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
		inline void shortcutPath(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& pathSC);
		inline void shortcutPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, std::vector<std::vector<Eigen::Vector3d>>& pathsSC); 
		inline void assignGuidePointsSemiCircle(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<std::pair<int, int>>& collisionSeg);
		inline bool hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints);
		inline bool hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints, Eigen::Vector3d& firstCollisionPos);
		inline bool hasDynamicCollisionTrajectory(const Eigen::MatrixXd& controlPoints);
		inline bool indexInCollisionSeg(const std::vector<std::pair<int, int>>& collisionSeg, int idx);
		inline void compareCollisionSeg(const std::vector<std::pair<int, int>>& prevCollisionSeg, const std::vector<std::pair<int, int>>& newCollisionSeg, std::vector<int>& newCollisionPoints, std::vector<int>& overlappedCollisionPoints);
		inline int findCollisionSegIndex(const std::vector<std::pair<int, int>>& collisionSeg, int idx);
		inline bool isControlPointRequireNewGuide(int controlPointIdx);
	};

	inline bool bsplineTraj::isGoalValid(){
		Eigen::Vector3d goal = this->optData_.controlPoints.col(this->optData_.controlPoints.cols()-1);
		if (this->map_->isInflatedOccupied(goal)){
			return false;
		}
		else{
			return true;
		}
	}

	inline bool bsplineTraj::checkCollisionLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
		for (double a=0.0; a<=1.0; a+=this->map_->getRes()){
			Eigen::Vector3d pMid = a * p1 + (1 - a) * p2;
			if (this->map_->isInflatedOccupied(pMid)){
				return true;
			}
		}
		return false;
	}

	inline void bsplineTraj::shortcutPath(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& pathSC){
		pathSC.clear();
		size_t ptr1 = 0; size_t ptr2 = 2;
		pathSC.push_back(path[ptr1]);

		if (path.size() == 1){
			return;
		}

		if (path.size() == 2){
			pathSC.push_back(path[1]);
			return;
		}


		while (ros::ok()){
			if (ptr2 > path.size() - 1){
				break;
			}

			Eigen::Vector3d p1 = path[ptr1]; Eigen::Vector3d p2 = path[ptr2];
			if (not this->checkCollisionLine(p1, p2)){
				if (ptr2 >= path.size() - 1){
					pathSC.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				pathSC.push_back(path[ptr2-1]);
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}
	}

	inline void bsplineTraj::shortcutPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, std::vector<std::vector<Eigen::Vector3d>>& pathsSC){
		pathsSC.clear();
		for (std::vector<Eigen::Vector3d> path : paths){
			std::vector<Eigen::Vector3d> pathSC;
			this->shortcutPath(path, pathSC);
			pathsSC.push_back(pathSC);
		}
	}

	inline void bsplineTraj::findGuidePointSemiCircle(int controlPointIdx, const std::pair<int, int>& seg, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint){
		int numControlpoints = seg.second - seg.first - 1; // number of segment
		double targetAngle = (controlPointIdx - seg.first) * PI_const/(numControlpoints+2); // angle incremental interval
		Eigen::Vector3d firstControlPoint = this->optData_.controlPoints.col(seg.first);
		Eigen::Vector3d controlPoint = this->optData_.controlPoints.col(controlPointIdx);
		Eigen::Vector3d direction = firstControlPoint - controlPoint;

		// calculate angle to each point in the shortcut path
		for (size_t i=0; i<path.size()-1; ++i){
			Eigen::Vector3d wpCurr = path[i];
			Eigen::Vector3d wpNext = path[i+1];
			double angleCurr = trajPlanner::angleBetweenVectors(direction, wpCurr - controlPoint);
			double angleNext = trajPlanner::angleBetweenVectors(direction, wpNext - controlPoint);
			if (targetAngle >= angleCurr and targetAngle <= angleNext){ // search point in this range
				double prevAngleDiff = 0.0;
				Eigen::Vector3d prevTempPoint;
				for (double a=1.0; a>=0.0; a-=0.1){
					Eigen::Vector3d tempPoint = a * wpCurr + (1-a) * wpNext;
					double tempAngle = trajPlanner::angleBetweenVectors(direction, tempPoint - controlPoint);
					double angleDiff = tempAngle - targetAngle;
					if (angleDiff == 0){ // we find the guide point
						guidePoint = tempPoint;
						return;
					}

					if (angleDiff * prevAngleDiff < 0){ // the guide point is between two
						double totalDiff = std::abs(angleDiff) + std::abs(prevAngleDiff);
						guidePoint = std::abs(prevAngleDiff)/totalDiff * (tempPoint - prevTempPoint) + prevTempPoint;
						return;
					}

					prevAngleDiff = angleDiff;
					prevTempPoint = tempPoint;
				}
			}

			// corner case: first and last point
			if (targetAngle < angleCurr and i==0){
				guidePoint = path[0];
				return;
			}	

			if (targetAngle > angleNext and i==path.size()-1){
				guidePoint = path.back();
				return;
			}
		}
	};

	inline bool bsplineTraj::hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints){
		std::vector<Eigen::Vector3d> trajectory = this->evalTraj();
		for (Eigen::Vector3d p : trajectory){
			if (p(2) > this->maxHeight_ or p(2) < this->minHeight_){
				return true;
			}
			bool hasCollision = this->map_->isInflatedOccupied(p);
			if (hasCollision){
				return true;
			}
		}
		return false;
	}

	inline bool bsplineTraj::hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints, Eigen::Vector3d& firstCollisionPos){
		std::vector<Eigen::Vector3d> trajectory = this->evalTraj();
		for (Eigen::Vector3d p : trajectory){
			bool hasCollision = false;
			hasCollision = this->map_->isInflatedOccupied(p);
			if (p(2) > this->maxHeight_ or p(2) < this->minHeight_){
				hasCollision = true;
			}
			if (hasCollision){
				firstCollisionPos = p;
				return true;
			}
		}
		return false;
	}

	inline bool bsplineTraj::hasDynamicCollisionTrajectory(const Eigen::MatrixXd& controlPoints){
		std::vector<Eigen::Vector3d> trajectory = this->evalTraj();
		for (Eigen::Vector3d p : trajectory){
			if (p(2) > this->maxHeight_ or p(2) < this->minHeight_){
				return true;
			}

			// iterate through each dynamic obstacles
			Eigen::Vector3d obstaclePos, obstacleSize, diff;
			double size, dist;
			for (size_t i=0; i<this->optData_.dynamicObstaclesPos.size(); ++i){
				obstaclePos = this->optData_.dynamicObstaclesPos[i];
				obstacleSize = this->optData_.dynamicObstaclesSize[i];
				size = pow(pow(obstacleSize(0)/2, 2) + pow(obstacleSize(1)/2, 2), 0.5);
				diff = p - obstaclePos;
				diff(2) = 0.0;
				dist = diff.norm() - size;
				if (dist < 0){
					return true;
				}
			} 
		}
		return false;
	}

	inline bool bsplineTraj::indexInCollisionSeg(const std::vector<std::pair<int, int>>& collisionSeg, int idx){
		for (std::pair<int, int> seg : collisionSeg){
			if (idx > seg.first and idx < seg.second){
				return true;
			}
		}
		return false;
	}

	inline void bsplineTraj::compareCollisionSeg(const std::vector<std::pair<int, int>>& prevCollisionSeg, const std::vector<std::pair<int, int>>& newCollisionSeg, 
												 std::vector<int>& newCollisionPoints, std::vector<int>& overlappedCollisionPoints){
		// compare two collision segments
		for (std::pair<int, int> newSeg : newCollisionSeg){
			for (int i=newSeg.first+1; i<=newSeg.second-1; ++i){
				if (this->indexInCollisionSeg(prevCollisionSeg, i)){
					overlappedCollisionPoints.push_back(i);
				}
				else{
					newCollisionPoints.push_back(i);
				}
			}
		}
	}

	inline int bsplineTraj::findCollisionSegIndex(const std::vector<std::pair<int, int>>& collisionSeg, int idx){
		int countIdx = 0;
		for (std::pair<int, int> seg : collisionSeg){
			if (idx > seg.first and idx < seg.second){
				return countIdx;
			}
			++countIdx;
		}
		ROS_ERROR("Index ERROR. Not exist.");
		return -1;
	}	

	inline bool bsplineTraj::isControlPointRequireNewGuide(int controlPointIdx){
		Eigen::Vector3d controlPoint = this->optData_.controlPoints.col(controlPointIdx);
		for (size_t i=0; i<this->optData_.guidePoints[controlPointIdx].size(); ++i){
			Eigen::Vector3d guidePoint = this->optData_.guidePoints[controlPointIdx][i];
			Eigen::Vector3d guideDirection = this->optData_.guideDirections[controlPointIdx][i];
			double dist = (controlPoint - guidePoint).dot(guideDirection);
			double distErr = this->dthresh_ - dist;
			if (distErr > 0){ // still can be adjusted by increasing distance weight
 				return false;
			}
		}
		return true;
	}


}

#endif
