/*
	FILE: bsplineTraj.cpp
	-----------------------------
	bspline trajectory solver implementation based on occupancy grid map 
*/
#include <trajectory_planner/bsplineTraj.h>

namespace trajPlanner{
	bsplineTraj::bsplineTraj(){}

	bsplineTraj::bsplineTraj(const ros::NodeHandle& nh) : nh_(nh){
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

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
		if (not this->nh_.getParam("bspline_traj/weight_smoothness", this->weightSmoothness_)){
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

		// optimization weight for dynamic obstacles
		if (not this->nh_.getParam("bspline_traj/weight_dynamic_obstacle", this->weightDynamicObstacle_)){
			this->weightDynamicObstacle_ = 1.0;
			cout << "[BsplineTraj]" << ": No weight dynamic obstacle. Use default: 2.0." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Weight dynamic obstacle: " << this->weightDynamicObstacle_ << endl;
		}	

		// min height
		if (not this->nh_.getParam("bspline_traj/min_height", this->minHeight_)){
			this->minHeight_ = 0.5;
			cout << "[BsplineTraj]" << ": No min height. Use default: 0.5 m." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Min height: " << this->minHeight_ << endl;
		}

		// max height
		if (not this->nh_.getParam("bspline_traj/max_height", this->maxHeight_)){
			this->maxHeight_ = 2.0;
			cout << "[BsplineTraj]" << ": No max height. Use default: 2.0 m." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Max Height: " << this->maxHeight_ << endl;
		}

		// uncertain aware factor
		if (not this->nh_.getParam("bspline_traj/uncertain_aware_factor", this->uncertainAwareFactor_)){
			this->uncertainAwareFactor_ = 2.0;
			cout << "[BsplineTraj]" << ": No uncertain aware factor. Use default: 2.0." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Uncertain aware factor: " << this->uncertainAwareFactor_ << endl;
		}

		// prediction horizon
		if (not this->nh_.getParam("bspline_traj/prediction_horizon", this->predHorizon_)){
			this->predHorizon_ = 2.0;
			cout << "[BsplineTraj]" << ": No prediction horizon. Use default: 2.0 s." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Prediction horizon: " << this->predHorizon_ << endl;
		}

		// distance threshold for dynamic obstacle
		if (not this->nh_.getParam("bspline_traj/distance_threshold_dynamic", this->distThreshDynamic_)){
			this->distThreshDynamic_ = 1.0;
			cout << "[BsplineTraj]" << ": No dynamic obstacle distance threshold. Use default: 2.0 s." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Dynamic obstacle distance: " << this->distThreshDynamic_ << endl;
		}
	}

	void bsplineTraj::registerPub(){
		this->controlPointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/control_points", 10);
		this->currTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/trajectory", 10);
		this->astarVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/astar_path", 10);
		this->guidePointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/guide_points_and_directions", 10);
		// this->visualizationWorker_ = std::thread(&bsplineTraj::startVisualization, this);
	}

	void bsplineTraj::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &bsplineTraj::visCB, this);
	}

	void bsplineTraj::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
		this->pathSearch_.reset(new AStar);
		this->pathSearch_->initGridMap(map, Eigen::Vector3i(100, 100, 100), this->minHeight_, this->maxHeight_);
	}

	bool bsplineTraj::updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition){
		if (path.poses.size() < 4){
			return false;
		}
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
		return true;
	}

	void bsplineTraj::updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize){
		this->optData_.dynamicObstaclesPos = obstaclesPos;
		this->optData_.dynamicObstaclesVel = obstaclesVel;
		this->optData_.dynamicObstaclesSize = obstaclesSize;
	}


	bool bsplineTraj::makePlan(){
		if (not this->isGoalValid()){
			cout << "[BsplineTraj]: Goal is not collision free. Force Return." << endl;
			return false;
		}

		ros::Time startTime = ros::Time::now();
		// step 1. find collision segment
		this->findCollisionSeg(this->optData_.controlPoints, this->collisionSeg_); // upodate collision seg

		// step 2. A* to find collision free path
		bool pathSearchSuccess = this->pathSearch(this->collisionSeg_, this->astarPaths_);
		if (not pathSearchSuccess){
			this->clear();
			return false;
		}

		// step 3. Assign guide point and directions
		this->assignGuidePointsSemiCircle(this->astarPaths_, this->collisionSeg_);

		// step 4. call solver
		bool optimizationSuccess = this->optimizeTrajectory();
		this->clear();
		if (not optimizationSuccess){
			return false;
		}

		ros::Time endTime = ros::Time::now();
		cout << "[BsplineTraj]: Total time: " << (endTime - startTime).toSec() << endl;
		return true;
	}

	bool bsplineTraj::makePlan(nav_msgs::Path& trajectory){
		bool success = this->makePlan();
		trajectory = this->evalTrajToMsg();
		this->adjustTrajFeasibility(trajectory);
		return success;
	}

	void bsplineTraj::clear(){
		this->optData_.guidePoints.clear();
		this->optData_.guideDirections.clear();
		this->optData_.dynamicObstaclesPos.clear();
		this->optData_.dynamicObstaclesVel.clear();
		this->optData_.dynamicObstaclesSize.clear();
		this->collisionSeg_.clear();
		this->astarPaths_.clear();
	}

	void bsplineTraj::findCollisionSeg(const Eigen::MatrixXd& controlPoints, std::vector<std::pair<int, int>>& collisionSeg){
		collisionSeg.clear();
		bool previousHasCollision = false;
		int endIdx = int((controlPoints.cols() - bsplineDegree - 1) - this->notCheckRatio_ * (controlPoints.cols() - 2*bsplineDegree));
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
					collisionSeg.push_back(seg);
				}
			}
			previousHasCollision = hasCollision;
		}
	}

	bool bsplineTraj::pathSearch(const std::vector<std::pair<int, int>>& collisionSeg, std::vector<std::vector<Eigen::Vector3d>>& paths){
		paths.clear();
		for (std::pair<int, int> seg : collisionSeg){
			Eigen::Vector3d pStart (this->optData_.controlPoints.col(seg.first));
			Eigen::Vector3d pEnd (this->optData_.controlPoints.col(seg.second));
			if (this->pathSearch_->AstarSearch(0.1, pStart, pEnd)){
				paths.push_back(this->pathSearch_->getPath());
			}
			else{
				// cout << "[BsplineTraj]: Path Search Error. Force return." << endl;
				return false; 
			}
		}	
		return true;
	}


	void bsplineTraj::assignGuidePointsSemiCircle(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<std::pair<int, int>>& collisionSeg){
		// step 1 shortcut A* path to get representative waypoints
		std::vector<std::vector<Eigen::Vector3d>> astarPathsSC;
		this->shortcutPaths(paths, astarPathsSC);

		// step 2: find corresponding path and collision segment
		std::pair<int, int> seg;
		std::vector<Eigen::Vector3d> path;
		Eigen::Vector3d guidePoint, guideDirection;

		for (size_t i=0; i<collisionSeg.size(); ++i){
			seg = collisionSeg[i];
			path = astarPathsSC[i];
			for (int controlPointIdx=seg.first+1; controlPointIdx<=seg.second; ++controlPointIdx){ // iterate through all collision control points
				this->findGuidePointSemiCircle(controlPointIdx, seg, path, guidePoint);
				this->optData_.guidePoints[controlPointIdx].push_back(guidePoint);
				guideDirection = (guidePoint - this->optData_.controlPoints.col(controlPointIdx))/(guidePoint - this->optData_.controlPoints.col(controlPointIdx)).norm();
				this->optData_.guideDirections[controlPointIdx].push_back(guideDirection);
			}
		}

	}

	bool bsplineTraj::isReguideRequired(std::vector<std::pair<int, int>>& reguideCollisionSeg){
		std::vector<std::pair<int, int>> prevCollisionSeg = this->collisionSeg_; // previous collision segment
		this->findCollisionSeg(this->optData_.controlPoints, this->collisionSeg_); // new collision segment

		std::vector<int> newCollisionPoints; // new collision controlpoints
		std::vector<int> overlappedCollisionPoints; // old collision points which both appears in previous segment and new segment 
		this->compareCollisionSeg(prevCollisionSeg, this->collisionSeg_, newCollisionPoints, overlappedCollisionPoints);

		std::set<int> collisionSegIndices;
		// for having new collision points: need to reguide and find its corresponding segment
		for (int newCollisionPointIdx : newCollisionPoints){
			// find its corresponding collision segment
			int segIdx = this->findCollisionSegIndex(this->collisionSeg_, newCollisionPointIdx);
			collisionSegIndices.insert(segIdx);
		}

		// for overlapped collision points: check its distance to current guide point: if larger than threshold, it means we need to reuide and find its corresponding segment
		for (int overlappedCollisionPointIdx : overlappedCollisionPoints){
			// determine whether needs new guide points
			if (this->isControlPointRequireNewGuide(overlappedCollisionPointIdx)){
				int segIdx = this->findCollisionSegIndex(this->collisionSeg_, overlappedCollisionPointIdx);
				collisionSegIndices.insert(segIdx);
			}
		}

		if (collisionSegIndices.size() == 0){
			return false;
		}

		// convert collisionSegIndices into reguideCollisionSeg
		for (int reguideSegIdx : collisionSegIndices){
			reguideCollisionSeg.push_back(this->collisionSeg_[reguideSegIdx]);
		}

		return true;
	}


	bool bsplineTraj::optimizeTrajectory(){
		int solverResult = this->optimize();
		double weightDistance0 = this->weightDistance_;
		double weightDynamicObstacle0 = this->weightDynamicObstacle_;
		int failCount = 0; 
		std::vector<vector<Eigen::Vector3d>> tempAstarPaths; // in case path search fail
		bool hasCollision = this->hasCollisionTrajectory(this->optData_.controlPoints);
		bool hasDynamicCollision = this->hasDynamicCollisionTrajectory(this->optData_.controlPoints);
		while (ros::ok()){
			if (not hasCollision and not hasDynamicCollision){
				break;
			}

			if (failCount >= 4){
				this->weightDistance_ = weightDistance0;
				this->weightDynamicObstacle_ = weightDynamicObstacle0;
				return false;
			}
			if (hasCollision){
				// need to determine whether the reguide is required
				std::vector<std::pair<int, int>> reguideCollisionSeg;
				if (this->isReguideRequired(reguideCollisionSeg)){ // this will update collision segment for next iteration
					bool pathSearchSuccess = this->pathSearch(reguideCollisionSeg, tempAstarPaths);
					
					if (pathSearchSuccess){
						this->astarPaths_ = tempAstarPaths; 
						this->assignGuidePointsSemiCircle(tempAstarPaths, reguideCollisionSeg);
					}
					else{
						this->weightDistance_ *= 2.0;
						++failCount;
					}
				}
				else{
					this->weightDistance_ *= 2.0; // no need reguide: this means weight is not big enough	
					++failCount;
				}
			}

			if (hasDynamicCollision){
				this->weightDynamicObstacle_ *= 2.0;
				cout << "weight dynamic obstacle: " << this->weightDynamicObstacle_ << endl;
			}

			solverResult = this->optimize();
			if (solverResult == lbfgs::LBFGS_CONVERGENCE or solverResult == lbfgs::LBFGSERR_MAXIMUMITERATION or
				solverResult == lbfgs::LBFGS_ALREADY_MINIMIZED or solverResult == lbfgs::LBFGS_STOP or solverResult == lbfgs::LBFGSERR_CANCELED){

			}
			else{
				this->weightDistance_ = weightDistance0;
				this->weightDynamicObstacle_ = weightDynamicObstacle0;
				return false;
			}
		}
		this->weightDistance_ = weightDistance0;
		this->weightDynamicObstacle_ = weightDynamicObstacle0;
		return true;
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

		int optimizeResult = lbfgs::lbfgs_optimize(variableNum, x, &finalCost, bsplineTraj::solverCostFunction, NULL, NULL, this, &solverParams);

		// check solvers' condition
		// if (optimizeResult == lbfgs::LBFGS_CONVERGENCE or optimizeResult == lbfgs::LBFGSERR_MAXIMUMITERATION or
		// 	optimizeResult == lbfgs::LBFGS_ALREADY_MINIMIZED or optimizeResult == lbfgs::LBFGS_STOP){
		// 	cout << "solver succeed!" << endl;
		// }
		// else if (optimizeResult == lbfgs::LBFGSERR_CANCELED){
		// 	cout << "solver force stopped" << endl;
		// }
		// else{
		// 	cout << "solver error" << endl;
		// }

		return optimizeResult;
	}

	void bsplineTraj::adjustTrajFeasibility(nav_msgs::Path& traj){
		std::vector<geometry_msgs::PoseStamped> rawTraj = traj.poses;
		std::vector<geometry_msgs::PoseStamped> newTraj;
		geometry_msgs::PoseStamped currPs, nextPs, insertPs;
		Eigen::Vector3d currP, nextP, direction, insertP;
		for (size_t i=0; i<rawTraj.size()-1; ++i){
			currPs = rawTraj[i];
			nextPs = rawTraj[i+1];
			currP = Eigen::Vector3d (currPs.pose.position.x, currPs.pose.position.y, currPs.pose.position.z);
			nextP = Eigen::Vector3d (nextPs.pose.position.x, nextPs.pose.position.y, nextPs.pose.position.z);

			newTraj.push_back(currPs);
			double dist = (nextP - currP).norm();
			direction = (nextP - currP)/dist;
			int numInsert = ceil(dist/this->ts_*this->maxVel_) - 1;
			for (int n=1; n<=numInsert; ++n){
				insertP = n * dist/(numInsert+1) * direction + currP;
				insertPs.pose.position.x = insertP(0);
				insertPs.pose.position.y = insertP(1);
				insertPs.pose.position.z = insertP(2);
				insertPs.pose.orientation = currPs.pose.orientation;
				newTraj.push_back(insertPs);
			}
		}
		newTraj.push_back(rawTraj.back());
		traj.poses = newTraj;
	}

	double bsplineTraj::solverCostFunction(void* func_data, const double* x, double* grad, const int n){
		trajPlanner::bsplineTraj* opt = reinterpret_cast<trajPlanner::bsplineTraj*>(func_data);
		double cost = opt->costFunction(x, grad, n);
		return cost;
	}

	double bsplineTraj::costFunction(const double* x, double* grad, const int n){
		memcpy(this->optData_.controlPoints.data()+3*bsplineDegree, x, n*sizeof(x[0])); // copy current optimized data into control points

		// get costs with their gradients
		double distanceCost, smoothnessCost, feasibilityCost, dynamicObstacleCost;
		Eigen::MatrixXd distanceGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		Eigen::MatrixXd smoothnessGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		Eigen::MatrixXd feasibilityGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		Eigen::MatrixXd dynamicObstacleGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		this->getDistanceCost(this->optData_.controlPoints, distanceCost, distanceGradient);
		this->getSmoothnessCost(this->optData_.controlPoints, smoothnessCost, smoothnessGradient);
		this->getFeasibilityCost(this->optData_.controlPoints, feasibilityCost, feasibilityGradient);
		this->getDynamicObstacleCost(this->optData_.controlPoints, dynamicObstacleCost, dynamicObstacleGradient);

		// total cost and gradient
		double totalCost = this->weightDistance_ * distanceCost + this->weightSmoothness_ * smoothnessCost + this->weightFeasibility_ * feasibilityCost + this->weightDynamicObstacle_ * dynamicObstacleCost;
		Eigen::MatrixXd totalGradient = this->weightDistance_ * distanceGradient + this->weightSmoothness_ * smoothnessGradient + this->weightFeasibility_ * feasibilityGradient + this->weightDynamicObstacle_ * dynamicObstacleGradient;

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
		double a, b, c;
		for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){ // for each control points
			for (size_t j=0; j<this->optData_.guidePoints[i].size(); ++j){ // each control points we check all the guide points and guide directions
				if (not this->map_->isUnknown(this->optData_.guidePoints[i][j])){
					a = 3.0 * this->dthresh_; b = -3 * pow(this->dthresh_, 2); c = pow(this->dthresh_, 3);
				}
				else{
					a = 3.0 * this->uncertainAwareFactor_ * this->dthresh_; b = -3 * pow(this->uncertainAwareFactor_ * this->dthresh_, 2); c = pow(this->uncertainAwareFactor_ * this->dthresh_, 3);
				}

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

	void bsplineTraj::getDynamicObstacleCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		cost = 0;
		if (this->optData_.dynamicObstaclesPos.size() == 0) return;

		// iterate through each control points
		double a, b, c;
		a = 3.0 * this->dthresh_; b = -3 * pow(this->dthresh_, 2); c = pow(this->dthresh_, 3);
		for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){
			Eigen::Vector3d controlPoint = controlPoints.col(i);
			for (size_t j=0; j<this->optData_.dynamicObstaclesPos.size(); ++j){
				double size = pow(pow(this->optData_.dynamicObstaclesSize[j](0)/2, 2) + pow(this->optData_.dynamicObstaclesSize[j](1), 2)/2, 0.5);
				Eigen::Vector3d obstaclesPos = this->optData_.dynamicObstaclesPos[j];
				Eigen::Vector3d diff = controlPoint - obstaclesPos;
				diff(2) = 0.0; // ignore z difference
				double dist = diff.norm() - size; // actual distance to obstacle (need to minus obstacle size)
				double distThresh = this->distThreshDynamic_;
				double distErr = distThresh - dist;
				Eigen::Vector3d grad = 2.0 * diff/diff.norm();


				if (distErr <= 0){
					// no punishment	
				}
				else if (distErr > 0 and distErr <= distThresh){
					cost += pow(distErr, 3);
					gradient.col(i) += -3.0 * pow(distErr, 2) * grad;
				}
				else if (distErr >= distThresh){
					cost += a * pow(distErr, 2) + b * distErr + c;
					gradient.col(i) += -(2 * a * distErr + b) * grad;
				}
			}
		}
	}

	void bsplineTraj::visCB(const ros::TimerEvent&){
		if (this->init_){
			this->publishControlPoints();
			this->publishCurrTraj();
			this->publishAstarPath();
			this->publishGuidePoints();
		}
	}

	// void bsplineTraj::startVisualization(){
	// 	ros::Rate r (10);
	// 	while (not this->init_){
	// 		r.sleep();
	// 	}

	// 	if (this->init_){
	// 		while (ros::ok()){
	// 			this->publishControlPoints();
	// 			this->publishCurrTraj();
	// 			this->publishAstarPath();
	// 			this->publishGuidePoints();
	// 			r.sleep();
	// 		}
	// 	}
	// }

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
				visualization_msgs::Marker pointG;
				for (size_t k=0; k<this->optData_.guidePoints[j].size(); ++k){
					Eigen::Vector3d p = this->optData_.guidePoints[j][k]; // only visualize the latest one
					pointG.header.frame_id = "map";
					pointG.header.stamp = ros::Time::now();
					pointG.ns = "guide_points";
					pointG.id = numGuidedPoints;
					pointG.type = visualization_msgs::Marker::SPHERE;
					pointG.action = visualization_msgs::Marker::ADD;
					pointG.pose.position.x = p(0);
					pointG.pose.position.y = p(1);
					pointG.pose.position.z = p(2);
					pointG.lifetime = ros::Duration(0.5);
					pointG.scale.x = 0.05;
					pointG.scale.y = 0.05;
					pointG.scale.z = 0.05;
					pointG.color.a = 1.0;
					pointG.color.r = 1.0;
					pointG.color.g = 0.0;
					pointG.color.b = 1.0;
					msgVec.push_back(pointG);				
					++numGuidedPoints;
				}
			}
		}

		// guide directions
		int numGuideDirections = 0;
		for (size_t i=0; i<this->collisionSeg_.size(); ++i){
			int collisionStartIdx = this->collisionSeg_[i].first+1;
			int collisionEndIdx = this->collisionSeg_[i].second-1;
			for (int j=collisionStartIdx; j<=collisionEndIdx; ++j){
				visualization_msgs::Marker arrow;
				Eigen::Vector3d p = this->optData_.controlPoints.col(j);
				for (size_t k=0; k<this->optData_.guideDirections[j].size(); ++k){
					Eigen::Vector3d pGuide = this->optData_.guidePoints[j][k]; // only visualize the latest one
					geometry_msgs::Point p1, p2;
					p1.x = p(0);
					p1.y = p(1);
					p1.z = p(2);
					p2.x = pGuide(0);
					p2.y = pGuide(1);
					p2.z = pGuide(2);
					std::vector<geometry_msgs::Point> pointsVec {p1, p2};
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
		}

		msg.markers = msgVec;
		this->guidePointsVisPub_.publish(msg);
	}


	geometry_msgs::PoseStamped bsplineTraj::getPose(double t){
		geometry_msgs::PoseStamped ps;
		this->bspline_ = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->ts_);
		Eigen::Vector3d p = this->bspline_.at(t);

		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);

		// orientation:
		trajPlanner::bspline velBspline = this->bspline_.getDerivative();
		Eigen::Vector3d vel = velBspline.at(t);
		ps.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, atan2(vel(1), vel(0)));

		return ps;
	}

	double bsplineTraj::getDuration(){
		this->bspline_ = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->ts_);
		return this->bspline_.getDuration();
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

	bool bsplineTraj::isCurrTrajValid(){
		if (not this->init_){
			return false;
		}
		return not this->hasCollisionTrajectory(this->optData_.controlPoints);
	}

	bool bsplineTraj::isCurrTrajValid(Eigen::Vector3d& firstCollisionPos){
		if (not this->init_){
			return false;
		}
		return not this->hasCollisionTrajectory(this->optData_.controlPoints, firstCollisionPos);
	}

	nav_msgs::Path bsplineTraj::evalTrajToMsg(){
		std::vector<Eigen::Vector3d> trajTemp = this->evalTraj();
		nav_msgs::Path traj;
		this->eigenPointsToPathMsg(trajTemp, traj);
		trajPlanner::bspline velBspline = this->bspline_.getDerivative();
		size_t i = 0;
		while (i < traj.poses.size()){
			double t = (double) i * this->ts_;
			Eigen::Vector3d vel = velBspline.at(t);
			traj.poses[i].pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, atan2(vel(1), vel(0)));
			++i;
		}

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