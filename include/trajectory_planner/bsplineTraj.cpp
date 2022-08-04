/*
	FILE: bsplineTraj.cpp
	-----------------------------
	bspline trajectory solver implementation based on occupancy grid map 
*/
#include <trajectory_planner/bsplineTraj.h>

namespace trajPlanner{
	bsplineTraj::bsplineTraj(){}

	void bsplineTraj::init(const ros::NodeHandle& nh){

		this->nh_ = nh;
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	void bsplineTraj::initParam(){
		if (not this->nh_.getParam("bspline_traj/timestep", this->ts_)){
			this->ts_ = 0.1;
			cout << "[BsplineTraj]" << ": No timestep. Use default: 0.1" << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": timestep: " << this->ts_ << endl;
		}
	}

	void bsplineTraj::registerPub(){
		this->controlPointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/control_points", 10);
		this->currTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/trajectory", 10);
		this->astarVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/astar_path", 10);
	}

	void bsplineTraj::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &bsplineTraj::visCB, this);
	}

	void bsplineTraj::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
		this->pathSearch_.reset(new AStar);
		this->pathSearch_->initGridMap(map, Eigen::Vector3i(100, 100, 100), 1.0);
	}

	void bsplineTraj::updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition){
		Eigen::MatrixXd controlPoints;
		std::vector<Eigen::Vector3d> curveFitPoints;
		this->pathMsgToEigenPoints(path, curveFitPoints);
		this->bspline_.parameterizeToBspline(this->ts_, curveFitPoints, startEndCondition, controlPoints);
		this->optData_.controlPoints = controlPoints;
		int controlPointNum = controlPoints.cols();
		this->optData_.guidePoints.resize(controlPointNum);
		this->optData_.guideDirections.resize(controlPointNum);
		this->optData_.findGuidePoint.resize(controlPointNum, false);
		this->init_ = true;
	}


	void bsplineTraj::makePlan(){
		// step 1. find collision segment
		cout << "start make plan" << endl;
		this->findCollisionSeg(this->optData_.controlPoints);
		cout << "collision segment founded" << endl;
		int countSegNum = 0;
		for (std::pair<int, int> seg : this->collisionSeg_){
			cout << "segment number: " << countSegNum << endl;
			cout << "start: " << seg.first << " end: " << seg.second << endl;
			++countSegNum;
		}

		// step 2. A* to find collision free path
		this->pathSearch();

		// step 3. Assign P, V pair
		this->assignPVpairs();

		// step 4. call solver
	}

	void bsplineTraj::findCollisionSeg(const Eigen::MatrixXd& controlPoints){
		this->collisionSeg_.clear();
		bool previousHasCollision = false;
		double checkRatio = 2/3;
		int endIdx = int((controlPoints.cols() - bsplineDegree - 1) - checkRatio * (controlPoints.cols() - 2*bsplineDegree));
		int pairStartIdx = bsplineDegree;
		int pairEndIdx = bsplineDegree;
		for (int i=bsplineDegree; i<endIdx; ++i){
			// check collision of each control point
			Eigen::Vector3d p = controlPoints.col(i);
			bool hasCollision = this->map_->isInflatedOccupied(p);

			// only if collision condition changes we need to change 
			if (hasCollision != previousHasCollision){
				if (hasCollision){// if has collision and collision status changes: this means this is a start of a collision segment. record the previous point.
					pairStartIdx = i-1;
				}
				else{ // if no collision and collision status changes: this means this is an end of a collision segment. record the previous point
					pairEndIdx = i+1;
					std::pair<int, int> seg {pairStartIdx, pairEndIdx};
					this->collisionSeg_.push_back(seg);
				}
			}
			previousHasCollision = hasCollision;
		}
	}

	void bsplineTraj::pathSearch(){
		this->astarPaths_.clear();
		for (std::pair<int, int> seg : this->collisionSeg_){
			Eigen::Vector3d pStart (this->optData_.controlPoints.col(seg.first));
			Eigen::Vector3d pEnd (this->optData_.controlPoints.col(seg.second));
			if (this->pathSearch_->AstarSearch(0.1, pStart, pEnd)){
				this->astarPaths_.push_back(this->pathSearch_->getPath());
			}
			else{
				cout << "[BsplineTraj]: Path Search Error. Force return." << endl;
				return; 
			}
		}	
	}

	void bsplineTraj::assignPVpairs(){
		for (int i=0; i<this->optData_.controlPoints.cols(); ++i){
			this->optData_.findGuidePoint[i] = false;
		}

		for (size_t i=0; i<this->collisionSeg_.size(); ++i){
			int collisionStartIdx = this->collisionSeg_[i].first;
			int collisionEndIdx = this->collisionSeg_[i].second;
			std::vector<Eigen::Vector3d> path = this->astarPaths_[i];
			for (int j=collisionStartIdx+1; j<collisionEndIdx; ++j){
				Eigen::Vector3d tangentDirection = this->optData_.controlPoints.col(j+1) - this->optData_.controlPoints.col(j-1);
				Eigen::Vector3d guidePoint;
				bool success = this->findGuidePointFromPath(this->optData_.controlPoints.col(j), tangentDirection, path, guidePoint);
				if (success){
					this->optData_.guidePoints[j].push_back(guidePoint);
					Eigen::Vector3d guideDirection = (guidePoint - this->optData_.controlPoints.col(j))/(guidePoint - this->optData_.controlPoints.col(j)).norm();
					this->optData_.guideDirections[j].push_back(guideDirection);
					this->optData_.findGuidePoint[j] = true; 
				}
				else{
					this->optData_.findGuidePoint[j] = false;
				}
			}
		}
	}


	void bsplineTraj::visCB(const ros::TimerEvent& ){
		if (this->init_){
			this->publishControlPoints();
			this->publishCurrTraj();
			this->publishAstarPath();
		}
	}

	void bsplineTraj::publishControlPoints(){
		visualization_msgs::MarkerArray msg;
		std::vector<visualization_msgs::Marker> pointVec;
		visualization_msgs::Marker point;
		int pointCount = 0;
		for (int i=0; i<this->optData_.controlPoints.cols(); ++i){
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "control_points";
			point.id = pointCount;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = this->optData_.controlPoints(0, i);
			point.pose.position.y = this->optData_.controlPoints(1, i);
			point.pose.position.z = this->optData_.controlPoints(2, i);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.1;
			point.scale.y = 0.1;
			point.scale.z = 0.1;
			point.color.a = 0.5;
			point.color.r = 1.0;
			point.color.g = 0.0;
			point.color.b = 0.0;
			pointVec.push_back(point);
			++pointCount;			
		}
		msg.markers = pointVec;
		this->controlPointsVisPub_.publish(msg);
	}

	void bsplineTraj::publishCurrTraj(){
		nav_msgs::Path trajMsg = this->evalTrajToMsg();
		this->currTrajVisPub_.publish(trajMsg);
	}

	void bsplineTraj::publishAstarPath(){
		visualization_msgs::MarkerArray msg;
		std::vector<visualization_msgs::Marker> pointVec;
		visualization_msgs::Marker point;
		int pointCount = 0;
		for (size_t i=0; i<this->astarPaths_.size(); ++i){
			std::vector<Eigen::Vector3d> path = this->astarPaths_[i];
			for (size_t j=0; j<path.size(); ++j){
				Eigen::Vector3d p = path[j];
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "astar_path";
				point.id = pointCount;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = p(0);
				point.pose.position.y = p(1);
				point.pose.position.z = p(2);
				point.lifetime = ros::Duration(0.5);
				point.scale.x = 0.1;
				point.scale.y = 0.1;
				point.scale.z = 0.1;
				point.color.a = 0.5;
				point.color.r = 0.0;
				point.color.g = 0.0;
				point.color.b = 1.0;
				pointVec.push_back(point);
				++pointCount;	
			}
		}
		msg.markers = pointVec;
		this->astarVisPub_.publish(msg);
	}

	std::vector<Eigen::Vector3d> bsplineTraj::evalTraj(){
		this->bspline_ = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->ts_);
		std::vector<Eigen::Vector3d> traj;
		Eigen::Vector3d p;
		for (double t=0; t<=this->bspline_.getDuration(); t+=this->ts_){
			p = this->bspline_.at(t);
			traj.push_back(p);
		}
		return traj;
	}	

	nav_msgs::Path bsplineTraj::evalTrajToMsg(){
		std::vector<Eigen::Vector3d> trajTemp = this->evalTraj();
		nav_msgs::Path traj;
		this->eigenPointsToPathMsg(trajTemp, traj);
		return traj;
	}

	void bsplineTraj::pathMsgToEigenPoints(const nav_msgs::Path& path, std::vector<Eigen::Vector3d>& points){
		Eigen::Vector3d p;
		for (size_t i=0; i<path.poses.size(); ++i){
			p(0) = path.poses[i].pose.position.x;
			p(1) = path.poses[i].pose.position.y;
			p(2) = path.poses[i].pose.position.z;
			points.push_back(p);
		}
	}

	void bsplineTraj::eigenPointsToPathMsg(const std::vector<Eigen::Vector3d>& points, nav_msgs::Path& path){
		std::vector<geometry_msgs::PoseStamped> pathVec;
		geometry_msgs::PoseStamped p;
		for (size_t i=0; i<points.size(); ++i){
			p.pose.position.x = points[i](0);
			p.pose.position.y = points[i](1);
			p.pose.position.z = points[i](2);
			pathVec.push_back(p);
		}
		path.poses = pathVec;
		path.header.frame_id = "map";
	}
}