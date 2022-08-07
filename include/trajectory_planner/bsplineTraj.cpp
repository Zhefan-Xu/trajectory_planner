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
		// timestep for bspline
		if (not this->nh_.getParam("bspline_traj/timestep", this->ts_)){
			this->ts_ = 0.1;
			cout << "[BsplineTraj]" << ": No timestep. Use default: 0.1 s." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": timestep: " << this->ts_ << " s." << endl;
		}

		// distance threshold for bspline optimization
		if (not this->nh_.getParam("bspline_traj/distance_threshold", this->dthresh_)){
			this->dthresh_ = 0.5;
			cout << "[BsplineTraj]" << ": No distance threshold. Use default: 0.5 m." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Distance threshold: " << this->dthresh_ << " m." << endl;
		}		

		// maximum velocity
		if (not this->nh_.getParam("bspline_traj/max_vel", this->maxVel_)){
			this->maxVel_ = 1.0;
			cout << "[BsplineTraj]" << ": No max velocity. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Maximum velocity: " << this->maxVel_ << " m." << endl;
		}	

		// maximum acceleration
		if (not this->nh_.getParam("bspline_traj/max_acc", this->maxAcc_)){
			this->maxAcc_ = 0.5;
			cout << "[BsplineTraj]" << ": No max acc. Use default: 0.5 m/s^2." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Max acc: " << this->maxAcc_ << " m/s^2."<< endl;
		}	

		// optimization weight for distance
		if (not this->nh_.getParam("bspline_traj/weight_distance", this->weightDistance_)){
			this->weightDistance_ = 0.5;
			cout << "[BsplineTraj]" << ": No weight distance. Use default: 2.0." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Weight distance: " << this->weightDistance_ << endl;
		}	

		// optimization weight for smoothness
		if (not this->nh_.getParam("bspline_traj/weight_distance", this->weightSmoothness_)){
			this->weightSmoothness_ = 1.0;
			cout << "[BsplineTraj]" << ": No weight smoothness. Use default: 1.0." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Weight smoothness: " << this->weightSmoothness_ << endl;
		}	

		// optimization weight for feasiblity
		if (not this->nh_.getParam("bspline_traj/weight_feasibility", this->weightFeasibility_)){
			this->weightFeasibility_ = 1.0;
			cout << "[BsplineTraj]" << ": No weight feasibility. Use default: 1.0." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Weight feasibility: " << this->weightFeasibility_ << endl;
		}	
	}

	void bsplineTraj::registerPub(){
		this->controlPointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/control_points", 10);
		this->currTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/trajectory", 10);
		this->astarVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/astar_path", 10);
		this->guidePointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/guide_points_and_directions", 10);
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
		ros::Time startTime = ros::Time::now();
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

		cout << "start solving..." << endl;
		// step 4. call solver
		this->optimizeTrajectory();

		ros::Time endTime = ros::Time::now();
		cout << "time: " << (endTime - startTime).toSec() << endl;
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


		for (size_t i=0; i<this->collisionSeg_.size(); ++i){ // go through each collision segment
			int collisionStartIdx = this->collisionSeg_[i].first;
			int collisionEndIdx = this->collisionSeg_[i].second;
			std::vector<Eigen::Vector3d> path = this->astarPaths_[i];
			bool successOnce = false; int successIdx = -1;
			for (int j=collisionStartIdx+1; j<collisionEndIdx; ++j){ // for each collision control point, we assign p v pair to it
				Eigen::Vector3d tangentDirection = this->optData_.controlPoints.col(j+1) - this->optData_.controlPoints.col(j-1);
				Eigen::Vector3d guidePoint;
				bool success = this->findGuidePointFromPath(this->optData_.controlPoints.col(j), tangentDirection, path, guidePoint);
				if (success){
					this->optData_.guidePoints[j].push_back(guidePoint);
					Eigen::Vector3d guideDirection = (guidePoint - this->optData_.controlPoints.col(j))/(guidePoint - this->optData_.controlPoints.col(j)).norm();
					this->optData_.guideDirections[j].push_back(guideDirection);
					this->optData_.findGuidePoint[j] = true; 
					successOnce = true; successIdx = j;
				}
				else{
					this->optData_.findGuidePoint[j] = false;
				}
			}

			// by the previous step some point will fail to get their guide point, so we need to assign guide point from its neighbor
			if (successOnce){
				for (int k=successIdx+1; k<collisionEndIdx; ++k){ // from success point forward to the end
					if (this->optData_.findGuidePoint[k] == false){ // this control point does not find a guide point
						this->optData_.guidePoints[k].push_back((this->optData_.guidePoints[k-1]).back()); // previous control points' guide point
						this->optData_.guideDirections[k].push_back((this->optData_.guideDirections[k-1]).back());
					}
				}

				for (int k=successIdx-1; k>collisionStartIdx; --k){ // from success point backward to the end
					if (this->optData_.findGuidePoint[k] == false){
						this->optData_.guidePoints[k].push_back((this->optData_.guidePoints[k+1]).back());
						this->optData_.guideDirections[k].push_back((this->optData_.guideDirections[k+1]).back());
					} 

				}
			}

		}
	}

	void bsplineTraj::optimizeTrajectory(){
		ros::Time startTime = ros::Time::now();
		// ros::Rate r (1);
		this->optimize();
		int count = 2;
		double weightDistance0 = this->weightDistance_;
		while (ros::ok() and this->hasCollisionTrajectory(this->optData_.controlPoints)){
			cout << "optimization: " << count << endl;
			this->optimize();

			// if not collision free, we increase weight for distance
			this->weightDistance_ *= 2.0;

			// this->publishCurrTraj();
			// this->publishControlPoints();
			// this->publishAstarPath();
			// this->publishGuidePoints();
			// cout << "optimization done." << endl;
			++count;
			// r.sleep();
		}
		this->weightDistance_ = weightDistance0;
		ros::Time endTime = ros::Time::now();
		cout << "total optimization time: " << (endTime - startTime).toSec() << endl;
	}

	int bsplineTraj::optimize(){
		// optimize information
		int variableNum = 3 * (this->optData_.controlPoints.cols() - 2*bsplineDegree);
		double x[variableNum]; int startID = bsplineDegree;
		memcpy(x, this->optData_.controlPoints.data()+3*startID, variableNum*sizeof(x[0])); // convert control point into solver's param
		double finalCost;

		// set up solver paramters
		lbfgs::lbfgs_parameter_t solverParams;
		lbfgs::lbfgs_load_default_parameters(&solverParams);
		solverParams.mem_size = 16;
		solverParams.max_iterations = 200;
		solverParams.g_epsilon = 0.01;

		ros::Time startTime = ros::Time::now();
		int optimizeResult = lbfgs::lbfgs_optimize(variableNum, x, &finalCost, bsplineTraj::solverCostFunction, NULL, bsplineTraj::solverForceStop, this, &solverParams);
		ros::Time endTime = ros::Time::now();
		cout << "final cost is: " << finalCost << endl;
		cout << "solver time: " << (endTime - startTime).toSec() << endl;

		// check solvers' condition
		if (optimizeResult == lbfgs::LBFGS_CONVERGENCE or optimizeResult == lbfgs::LBFGSERR_MAXIMUMITERATION or
			optimizeResult == lbfgs::LBFGS_ALREADY_MINIMIZED or optimizeResult == lbfgs::LBFGS_STOP){
			cout << "solver succeed!" << endl;
		}
		else if (optimizeResult == lbfgs::LBFGSERR_CANCELED){
			cout << "solver force stopped" << endl;
		}
		else{
			cout << "solver error" << endl;
		}

		return optimizeResult;
	}

	double bsplineTraj::solverCostFunction(void* func_data, const double* x, double* grad, const int n){
		trajPlanner::bsplineTraj* opt = reinterpret_cast<trajPlanner::bsplineTraj*>(func_data);
		double cost = opt->costFunction(x, grad, n);
		return cost;
	}

	int bsplineTraj::solverForceStop(void* func_data, const double* x, const double* grad, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls){
		// TODO implement force stop
		return 0;
	}

	double bsplineTraj::costFunction(const double* x, double* grad, const int n){
		memcpy(this->optData_.controlPoints.data()+3*bsplineDegree, x, n*sizeof(x[0])); // copy current optimized data into control points

		// get costs with their gradients
		double distanceCost, smoothnessCost, feasibilityCost;
		Eigen::MatrixXd distanceGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		Eigen::MatrixXd smoothnessGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		Eigen::MatrixXd feasibilityGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		this->getDistanceCost(this->optData_.controlPoints, distanceCost, distanceGradient);
		this->getSmoothnessCost(this->optData_.controlPoints, smoothnessCost, smoothnessGradient);
		this->getFeasibilityCost(this->optData_.controlPoints, feasibilityCost, feasibilityGradient);

		// total cost and gradient
		double totalCost = this->weightDistance_ * distanceCost + this->weightSmoothness_ * smoothnessCost + this->weightFeasibility_ * feasibilityCost;
		Eigen::MatrixXd totalGradient = this->weightDistance_ * distanceGradient + this->weightSmoothness_ * smoothnessGradient + this->weightFeasibility_ * feasibilityGradient;

		// update gradient
		memcpy(grad, totalGradient.data()+3*bsplineDegree, n*sizeof(grad[0]));
		return totalCost;
	}

	void bsplineTraj::getDistanceCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		/* cost function: 
		 	0,   if d_thresh - d <= 0
		 	d^3, if 0<= d_thresh - d <=  d_thesh
		 	3*d_thresh*(d-d_thresh)^2 - 3*d_thresh^2*(d-d_thresh) + (d-d_thresh)^3 if d_thresh - d  >=d_thresh
			
		*/

		cost = 0.0; // initialize cost
		Eigen::Vector3d controlPoint;
		std::vector<Eigen::Vector3d>  guidePoints, guideDirections;
		double a = 3.0 * this->dthresh_; double b = -3 * pow(this->dthresh_, 2); double c = pow(this->dthresh_, 3);
		for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){ // for each control points
			for (size_t j=0; j<this->optData_.guidePoints[i].size(); ++j){ // each control points we check all the guide points and guide directions
				double dist = (controlPoints.col(i) - this->optData_.guidePoints[i][j]).dot(this->optData_.guideDirections[i][j]);
				double distErr = this->dthresh_ - dist;
				if (distErr <= 0){
					// no punishment	
				}
				else if (distErr > 0 and distErr <= this->dthresh_){
					cost += pow(distErr, 3);
					gradient.col(i) += -3.0 * pow(distErr, 2) * this->optData_.guideDirections[i][j];
				}
				else if (distErr >= this->dthresh_){
					cost += a * pow(distErr, 2) + b * distErr + c;
					gradient.col(i) += -(2 * a * distErr + b) * this->optData_.guideDirections[i][j];
				}
			}
		}
	}

	void bsplineTraj::getSmoothnessCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		// minimize jerk of the trajectory
		cost = 0.0;
		Eigen::Vector3d jerk, gradTemp;
		for (int i=0; i<controlPoints.cols()-bsplineDegree; ++i){
			jerk = controlPoints.col(i+3) - 3 * controlPoints.col(i+2) + 3 * controlPoints.col(i+1) - controlPoints.col(i);
			cost += jerk.squaredNorm();
			gradTemp = 2.0 * jerk;
			/* jerk gradient */
			gradient.col(i) += -gradTemp;
			gradient.col(i+1) += 3.0 * gradTemp;
			gradient.col(i+2) += -3.0 * gradTemp;
			gradient.col(i+3) += gradTemp;
		}
	}

	void bsplineTraj::getFeasibilityCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		// velocity and acceleration cost
		cost = 0.0;

		// velocity cost
		double tsInvSqr = 1/pow(this->ts_, 2); // for balancing velocity and acceleration scales
		Eigen::Vector3d vi;
		for (int i=0; i<controlPoints.cols()-1; ++i){
			vi = (controlPoints.col(i+1) - controlPoints.col(i))/this->ts_;
			for (int j=0; j<3; ++j){ // 3 axis
				if (vi(j) > this->maxVel_){
					cost += pow(vi(j) - this->maxVel_, 2) * tsInvSqr;
					gradient(j, i) += -2 * (vi(j) - this->maxVel_)/(this->ts_) * tsInvSqr;
					gradient(j, i+1) += 2 * (vi(j) - this->maxVel_)/(this->ts_) * tsInvSqr;
				}
				else if (vi(j) < -this->maxVel_){
					cost += pow(vi(j) + this->maxVel_, 2) * tsInvSqr;
					gradient(j, i) += -2 * (vi(j) + this->maxVel_)/(this->ts_) * tsInvSqr;
					gradient(j, i+1) += 2 * (vi(j) + this->maxVel_)/(this->ts_) * tsInvSqr;
				}				
			}
		}

		// acceleration cost
		Eigen::Vector3d ai;
		for (int i=0; i<controlPoints.cols()-2; ++i){
			ai = (controlPoints.col(i+2) - 2 * controlPoints.col(i+1) + controlPoints.col(i)) * tsInvSqr;
			for (int j=0; j<3; ++j){
				if (ai(j) > this->maxAcc_){
					cost += pow(ai(j) - this->maxAcc_, 2);
					gradient(j, i) += 2 * (ai(j) - this->maxAcc_) * tsInvSqr;
					gradient(j, i+1) += -4 * (ai(j) - this->maxAcc_) * tsInvSqr;
					gradient(j, i+2) += 2 * (ai(j) - this->maxAcc_) * tsInvSqr;
				}
				else if (ai(j) < -this->maxAcc_){
					cost += pow(ai(j) + this->maxAcc_, 2);
					gradient(j, i) += 2 * (ai(j) + this->maxAcc_) * tsInvSqr;
					gradient(j, i+1) += -4 * (ai(j) + this->maxAcc_) * tsInvSqr;
					gradient(j, i+2) += 2 * (ai(j) + this->maxAcc_) * tsInvSqr;
				}
			}
		}
	}

	void bsplineTraj::visCB(const ros::TimerEvent& ){
		if (this->init_){
			this->publishControlPoints();
			this->publishCurrTraj();
			this->publishAstarPath();
			this->publishGuidePoints();
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

	void bsplineTraj::publishGuidePoints(){
		visualization_msgs::MarkerArray msg;
		std::vector<visualization_msgs::Marker> msgVec;
		visualization_msgs::Marker point;

		// collision control points
		int numCollisionControlPoints = 0;
		for (size_t i=0; i<this->collisionSeg_.size(); ++i){
			int collisionStartIdx = this->collisionSeg_[i].first+1;
			int collisionEndIdx = this->collisionSeg_[i].second-1;
			for (int j=collisionStartIdx; j<=collisionEndIdx; ++j){
				Eigen::Vector3d p = this->optData_.controlPoints.col(j);
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "collision_control_point";
				point.id = numCollisionControlPoints;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = p(0);
				point.pose.position.y = p(1);
				point.pose.position.z = p(2);
				point.lifetime = ros::Duration(0.5);
				point.scale.x = 0.05;
				point.scale.y = 0.05;
				point.scale.z = 0.05;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 0.0;
				point.color.b = 0.0;
				msgVec.push_back(point);				
				++numCollisionControlPoints;
			}
		}

		// guide points
		int numGuidedPoints = 0;
		for (size_t i=0; i<this->collisionSeg_.size(); ++i){
			int collisionStartIdx = this->collisionSeg_[i].first+1;
			int collisionEndIdx = this->collisionSeg_[i].second-1;
			for (int j=collisionStartIdx; j<=collisionEndIdx; ++j){
				Eigen::Vector3d p = this->optData_.guidePoints[j].back(); // only visualize the latest one
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "guide_points";
				point.id = numGuidedPoints;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = p(0);
				point.pose.position.y = p(1);
				point.pose.position.z = p(2);
				point.lifetime = ros::Duration(0.5);
				point.scale.x = 0.05;
				point.scale.y = 0.05;
				point.scale.z = 0.05;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 0.0;
				point.color.b = 1.0;
				msgVec.push_back(point);				
				++numGuidedPoints;
			}
		}

		// guide directions
		int numGuideDirections = 0;
		visualization_msgs::Marker arrow;
		for (size_t i=0; i<this->collisionSeg_.size(); ++i){
			int collisionStartIdx = this->collisionSeg_[i].first+1;
			int collisionEndIdx = this->collisionSeg_[i].second-1;
			for (int j=collisionStartIdx; j<=collisionEndIdx; ++j){
				Eigen::Vector3d p = this->optData_.controlPoints.col(j);
				Eigen::Vector3d pGuide = this->optData_.guidePoints[j].back(); // only visualize the latest one
				geometry_msgs::Point p1, p2;
				p1.x = p(0);
				p1.y = p(1);
				p1.z = p(2);
				p2.x = pGuide(0);
				p2.y = pGuide(1);
				p2.z = pGuide(2);
				std::vector<geometry_msgs::Point> pointsVec {p1, p2};
				// Eigen::Vector3d direction = p_ - p;
				arrow.points = pointsVec;
				arrow.header.frame_id = "map";
				arrow.header.stamp = ros::Time::now();
				arrow.ns = "guide_direction";
				arrow.id = numGuideDirections;
				arrow.type = visualization_msgs::Marker::ARROW;
				arrow.lifetime = ros::Duration(0.5);
				arrow.scale.x = 0.05;
				arrow.scale.y = 0.05;
				arrow.scale.z = 0.05;
				arrow.color.a = 0.5;
				arrow.color.r = 0.0;
				arrow.color.g = 1.0;
				arrow.color.b = 0.0;
				msgVec.push_back(arrow);				
				++numGuideDirections;
			}
		}

		msg.markers = msgVec;
		this->guidePointsVisPub_.publish(msg);
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