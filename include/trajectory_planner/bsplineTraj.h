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
#include <limits>

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
		ros::Publisher inputTrajPub_;
		ros::Publisher inputTrajPointPub_;


		// bspline
		double controlPointDistance_ = 0.25; // magic number 0.2, 0.25 (new)
		double controlPointsTs_ = 0.2; // magic number 0.1, 0.2 (new)
		trajPlanner::bspline bspline_; // this is used to evaluate bspline. not for optimization
		trajPlanner::optData optData_; // all optimization information including control points
		double ts_; // original time step and adjusted time step (this is for control points)
		double dthresh_;
		double maxVel_;
		double maxAcc_;
		double weightDistance_;
		double weightSmoothness_;
		double weightFeasibility_;
		double weightDynamicObstacle_;
		double notCheckRatio_ = 0.0;
		// double notCheckRatio_ = 1.0/3.0;
		bool planInZAxis_;
		double minHeight_;
		double maxHeight_;
		double uncertainAwareFactor_;
		double predHorizon_; // prediction horizon for dynamic obstacles in seconds
		double distThreshDynamic_; // distance threshold to keep with dynamic obstacles
		double maxPathLength_;
		Eigen::Vector3d maxObstacleSize_;

		// occupancy grid map and path search
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<AStar> pathSearch_;

		std::vector<std::pair<int, int>> collisionSeg_;
		std::vector<std::vector<Eigen::Vector3d>> astarPaths_;
		std::vector<std::vector<Eigen::Vector3d>> astarPathsSC_; // shortcut paths

		// flag
		bool init_ = false;


		// time reparameterization
		double linearFactor_ = 1.0;

		// visualization
		std::vector<Eigen::Vector3d> inputPathVis_;

	public:
		// std::thread visualizationWorker_;

		bsplineTraj();
		bsplineTraj(const ros::NodeHandle& nh);
		void init(const ros::NodeHandle& nh);
		void initParam();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map); // update occuapncy grid map
		void updateMaxVel(double maxVel);
		void updateMaxAcc(double maxAcc);
		bool inputPathCheck(const nav_msgs::Path & path, nav_msgs::Path& adjustedPath, double dt, double& finalTime);
		bool fillPath(const nav_msgs::Path& path, nav_msgs::Path& adjustedPath);
		bool updatePath(const nav_msgs::Path& adjustedPath, const std::vector<Eigen::Vector3d>& startEndConditions);
		void updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize); // position, velocity, size

		bool makePlan();
		bool makePlan(nav_msgs::Path& trajectory, bool yaw=true);
		void clear();
		void findCollisionSeg(const Eigen::MatrixXd& controlPoints, std::vector<std::pair<int, int>>& collisionSeg); // find collision segment of current control points
		bool pathSearch(std::vector<std::pair<int, int>>& collisionSeg, std::vector<std::vector<Eigen::Vector3d>>& paths);
		void assignGuidePointsSemiCircle(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<std::pair<int, int>>& collisionSeg);
		void assignGuidePointDynamicObstacle();
		bool isReguideRequired(std::vector<std::pair<int, int>>& reguideCollisionSeg);
		bool optimizeTrajectory();
		int optimize(); // optimize once
		void adjustPathLength(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& adjustedPath);
		void adjustPathLengthDirect(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& adjustedPath);

		// cost functions
		static double solverCostFunction(void* func_data, const double* x, double* grad, const int n); // deal with solver
		double costFunction(const double* x, double* grad, const int n);
		void getDistanceCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // collision
		void getSmoothnessCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // trajectory
		void getFeasibilityCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // velocity and acceleration
		void getDynamicObstacleCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient); // dynamic obstacle

		// time reparameterization
		void linearFeasibilityReparam();
		double getLinearReparamTime(double t);
		double getLinearFactor();


		// visualization
		// void startVisualization();
		void visCB(const ros::TimerEvent&);
		void publishControlPoints();
		void publishCurrTraj();
		void publishAstarPath();
		void publishGuidePoints();
		void publishInputTraj();


		// user functions
		double getInitTs(); // initial sample time
		double getControlPointTs();
		double getControlPointDist();
		trajPlanner::bspline getTrajectory();
		geometry_msgs::PoseStamped getPose(double t, bool yaw=true);
		double getDuration();
		double getTimestep();
		Eigen::MatrixXd getControlPoints();
		bool isCurrTrajValid();
		bool isCurrTrajValid(Eigen::Vector3d& firstCollisionPos);
		void writeCurrentTrajInfo(const std::string& filePath, double dt);

		// helper functionin
		std::vector<Eigen::Vector3d> evalTraj();
		std::vector<Eigen::Vector3d> evalTraj(double dt);
		nav_msgs::Path evalTrajToMsg(bool yaw=true); // evaluate current trajectory based on the control point
		nav_msgs::Path evalTrajToMsg(double dt, bool yaw=true);
		void pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points);
		void eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path);

		// inline function
		// inline bool isGoalValid();
		inline bool checkCollisionLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
		inline void shortcutPath(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& pathSC);
		inline void shortcutPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, std::vector<std::vector<Eigen::Vector3d>>& pathsSC); 
		inline bool findGuidePointSemiCircle(int controlPointIdx, const std::pair<int, int>& seg, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint);
		inline bool hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints);
		inline bool hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints, Eigen::Vector3d& firstCollisionPos);
		inline bool hasDynamicCollisionTrajectory(const Eigen::MatrixXd& controlPoints);
		inline bool indexInCollisionSeg(const std::vector<std::pair<int, int>>& collisionSeg, int idx);
		inline void compareCollisionSeg(const std::vector<std::pair<int, int>>& prevCollisionSeg, const std::vector<std::pair<int, int>>& newCollisionSeg, std::vector<int>& newCollisionPoints, std::vector<int>& overlappedCollisionPoints);
		inline int findCollisionSegIndex(const std::vector<std::pair<int, int>>& collisionSeg, int idx);
		inline bool isControlPointRequireNewGuide(int controlPointIdx);
		inline double signHelper(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);
		inline bool isInTriangle(const Eigen::Vector2d& pt, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const Eigen::Vector2d& v3);
		inline int isInDistanceField(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& obstaclesVel, const Eigen::Vector3d& p, double radius);
		inline void getDynamicCostAndGradCircle(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& p, double radius, double& distErrs, Eigen::Vector3d& grad);
		inline void getDynamicCostAndGradPolygon(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& obstaclesVel, const Eigen::Vector3d& p, double radius, double& distErr, Eigen::Vector3d& grad);

	};

	// inline bool bsplineTraj::isGoalValid(){
	// 	trajPlanner::bspline traj (bsplineDegree, this->optData_.controlPoints, this->controlPointsTs_);
	// 	Eigen::Vector3d goal = traj.at(traj.getDuration());
	// 	// Eigen::Vector3d goal = this->optData_.controlPoints.col(this->optData_.controlPoints.cols()-1);
	// 	if (this->map_->isInflatedOccupied(goal)){
	// 		cout << "[BsplineTraj]: Invalid goal position is: (" << goal(0) << ", " << goal(1) << ", " << goal(2) << ")." << endl;
	// 		return false;
	// 	}
	// 	else{
	// 		return true;
	// 	}
	// }

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

	inline bool bsplineTraj::findGuidePointSemiCircle(int controlPointIdx, const std::pair<int, int>& seg, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& guidePoint){
		double minAngle = PI_const*0.0/4.0; 
		double maxAngle = PI_const*4.0/4.0;
		int numControlpoints = seg.second - seg.first - 1; // number of segment 
		
		double targetAngle; // target guide direction angle
		Eigen::Vector3d psudoControlPoint; // projected point
		if (numControlpoints != 0){
			int controlPointOrder = controlPointIdx - seg.first; // sequence order of collision control point
			targetAngle = (controlPointIdx - seg.first) * PI_const/(numControlpoints+2); // angle incremental interval
			targetAngle = std::min(std::max(minAngle, targetAngle), maxAngle);

			double ratio = double(controlPointOrder)/double(numControlpoints+1.0);
			psudoControlPoint = ratio * (path.back() - path[0]) + path[0];
		}
		else{ // line collision
			targetAngle = PI_const/2.0;
			psudoControlPoint = (path[0] + path.back())/2.0;
		}
		Eigen::Vector3d direction = path[0] - psudoControlPoint;; // guide direction


		// calculate angle to each point in the shortcut path
		for (size_t i=0; i<path.size()-1; ++i){
			Eigen::Vector3d wpCurr = path[i];
			Eigen::Vector3d wpNext = path[i+1];
			double angleCurr = trajPlanner::angleBetweenVectors(direction, wpCurr - psudoControlPoint);
			double angleNext = trajPlanner::angleBetweenVectors(direction, wpNext - psudoControlPoint);

			if (targetAngle >= angleCurr and targetAngle <= angleNext){ // search point in this range
				double prevAngleDiff = 0.0;
				Eigen::Vector3d prevTempPoint;
				for (double a=1.0; a>=0.0; a-=0.1){
					Eigen::Vector3d tempPoint = a * wpCurr + (1-a) * wpNext;
					double tempAngle = trajPlanner::angleBetweenVectors(direction, tempPoint - psudoControlPoint);
					double angleDiff = tempAngle - targetAngle;
					if (angleDiff == 0){ // we find the guide point
						guidePoint = tempPoint;
						return true;
					}

					if (angleDiff * prevAngleDiff < 0){ // the guide point is between two
						double totalDiff = std::abs(angleDiff) + std::abs(prevAngleDiff);
						guidePoint = std::abs(prevAngleDiff)/totalDiff * (tempPoint - prevTempPoint) + prevTempPoint;
						return true;
					}

					prevAngleDiff = angleDiff;
					prevTempPoint = tempPoint;
				}
			}
		}
		return false;
	};


	inline bool bsplineTraj::hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints){
		std::vector<Eigen::Vector3d> trajectory;
		Eigen::Vector3d p;
		trajPlanner::bspline bsplineTraj = trajPlanner::bspline (bsplineDegree, controlPoints, this->controlPointsTs_);
		
		double ts = this->map_->getRes()/(this->maxVel_)/2.0;
		for (double t=0; t<=(1.0-this->notCheckRatio_)*bsplineTraj.getDuration(); t+=ts){
			p = bsplineTraj.at(t);
			trajectory.push_back(p);
		}		

		for (Eigen::Vector3d p : trajectory){
			bool hasCollision = this->map_->isInflatedOccupied(p);
			if (hasCollision){
				return true;
			}
		}
		return false;
	}

	inline bool bsplineTraj::hasCollisionTrajectory(const Eigen::MatrixXd& controlPoints, Eigen::Vector3d& firstCollisionPos){
		std::vector<Eigen::Vector3d> trajectory = this->evalTraj();
		for (int i=0; i<(1.0-this->notCheckRatio_)*int(trajectory.size()); ++i){
			Eigen::Vector3d p = trajectory[i];
			bool hasCollision = false;
			hasCollision = this->map_->isInflatedOccupied(p);
			// if (p(2) > this->maxHeight_ or p(2) < this->minHeight_){
			// 	hasCollision = true;
			// }
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
			// if (p(2) > this->maxHeight_ or p(2) < this->minHeight_){
			// 	return true;
			// }

			// iterate through each dynamic obstacles
			Eigen::Vector3d obstaclePos, obstacleSize, diff;
			double size, dist;
			for (size_t i=0; i<this->optData_.dynamicObstaclesPos.size(); ++i){
				obstaclePos = this->optData_.dynamicObstaclesPos[i];
				obstacleSize = this->optData_.dynamicObstaclesSize[i];
				// size = pow(pow(obstacleSize(0)/2, 2) + pow(obstacleSize(1)/2, 2), 0.5);
				size = std::min(obstacleSize(0)/2, obstacleSize(1)/2);
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
			if (idx >= seg.first and idx <= seg.second){
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
			bool lineCollision = (newSeg.second - newSeg.first - 1 == 0);
			if (lineCollision){
				for (int i=newSeg.first; i<=newSeg.second; ++i){
					if (this->indexInCollisionSeg(prevCollisionSeg, i)){
						overlappedCollisionPoints.push_back(i);
					}
					else{
						newCollisionPoints.push_back(i);
					}
				}				
			}
		}
	}

	inline int bsplineTraj::findCollisionSegIndex(const std::vector<std::pair<int, int>>& collisionSeg, int idx){
		int countIdx = 0;
		for (std::pair<int, int> seg : collisionSeg){
			if (idx >= seg.first and idx <= seg.second){
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

	inline double bsplineTraj::signHelper(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3){
		return (p1(0) - p3(0)) * (p2(1) - p3(1)) - (p2(0) - p3(0)) * (p1(1) - p3(1));
	}

	inline bool bsplineTraj::isInTriangle(const Eigen::Vector2d& pt, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const Eigen::Vector2d& v3){
		/*
			https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
		*/
		double d1 = this->signHelper(pt, v1, v2);
		double d2 = this->signHelper(pt, v2, v3);
		double d3 = this->signHelper(pt, v1, v3);

		bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
		bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
		return !(hasNeg && hasPos);
	}

	
	inline int bsplineTraj::isInDistanceField(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& obstaclesVel, const Eigen::Vector3d& p, double radius){
		// 0: not in the distance field
		// 1: in the polygon region
		// 2: in the circle region

		// 1. first find two triangles' vertex
		Eigen::Vector3d predPos = obstaclePos + obstaclesVel * this->predHorizon_; 
		Eigen::Vector2d pTest (p(0), p(1));
		Eigen::Vector2d pt1 (obstaclePos(0), obstaclePos(1)); // shared one
		Eigen::Vector2d pt2 (predPos(0), predPos(1)); // shared one
		double length = (pt2 - pt1).norm();
		double angle = std::acos(radius/length);
		Eigen::Matrix2d rotA, rotB;
		rotA << cos(angle), -sin(angle), sin(angle), cos(angle);
		rotB << cos(-angle), -sin(-angle), sin(-angle), cos(-angle);
		Eigen::Vector2d unit12 = (pt2 - pt1)/length;
		Eigen::Vector2d unit13A =  rotA * unit12;
		Eigen::Vector2d unit13B =  rotB * unit12;
		Eigen::Vector2d pt3A = pt1 + radius * unit13A; 
		Eigen::Vector2d pt3B = pt1 + radius * unit13B;


		// in polygon region
		if (this->isInTriangle(pTest, pt1, pt2, pt3A) or this->isInTriangle(pTest, pt1, pt2, pt3B)){
			return 1;
		}

		// in circular region
		if ((pTest - pt1).norm() < radius){
			return 2;
		}

		return 0;
	}

	inline void bsplineTraj::getDynamicCostAndGradCircle(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& p, double radius, double& distErr, Eigen::Vector3d& grad){
		Eigen::Vector2d pTest (p(0), p(1));
		Eigen::Vector2d pt1 (obstaclePos(0), obstaclePos(1)); // shared one
		double distToObstacle = (pTest - pt1).norm();
		distErr = radius - distToObstacle;
		Eigen::Vector2d gradTemp = (pt1 - pTest)/(pt1 - pTest).norm(); 
		grad = Eigen::Vector3d (gradTemp(0), gradTemp(1), 0.0);
	}

	inline void bsplineTraj::getDynamicCostAndGradPolygon(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& obstaclesVel, const Eigen::Vector3d& p, double radius, double& distErr, Eigen::Vector3d& grad){
		Eigen::Vector3d predPos = obstaclePos + obstaclesVel * this->predHorizon_; 
		Eigen::Vector2d pTest (p(0), p(1));
		Eigen::Vector2d pt1 (obstaclePos(0), obstaclePos(1)); // shared one
		Eigen::Vector2d pt2 (predPos(0), predPos(1)); // shared one
		double length = (pt2 - pt1).norm();
		double angle = std::acos(radius/length);
		Eigen::Matrix2d rotA, rotB;
		rotA << cos(angle), -sin(angle), sin(angle), cos(angle);
		rotB << cos(-angle), -sin(-angle), sin(-angle), cos(-angle);
		Eigen::Vector2d unit12 = (pt2 - pt1)/length;
		Eigen::Vector2d unit13A =  rotA * unit12;
		Eigen::Vector2d unit13B =  rotB * unit12;
		Eigen::Vector2d pt3A = pt1 + radius * unit13A; 
		Eigen::Vector2d pt3B = pt1 + radius * unit13B;
		Eigen::Vector2d pointToPt2 = pt2 - pTest;

		if (this->isInTriangle(pTest, pt1, pt2, pt3A)){
			distErr = pointToPt2.dot(unit13A);
			Eigen::Vector2d gradTemp = unit13A;
			grad = Eigen::Vector3d (gradTemp(0), gradTemp(1), 0.0);
			return;
		}

		if (this->isInTriangle(pTest, pt1, pt2, pt3B)){
			distErr = pointToPt2.dot(unit13B);
			Eigen::Vector2d gradTemp = unit13B;
			grad = Eigen::Vector3d (gradTemp(0), gradTemp(1), 0.0);
			return;
		}
		
	}

}

#endif
