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

		// whether plan in z direction or not
		if (not this->nh_.getParam("bspline_traj/plan_in_z_axis", this->planInZAxis_)){
			this->planInZAxis_ = true;
			cout << "[BsplineTraj]" << ": No plan in z axis. Use default: true." << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Plan in z axis: " << this->planInZAxis_ << endl;
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

		// maximum path length for bspline trajectory optimization
		if (not this->nh_.getParam("bspline_traj/max_path_length", this->maxPathLength_)){
			this->maxPathLength_ = 7.0;
			cout << "[BsplineTraj]" << ": No maximum path length. Use default: 7.0 m" << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": Maximum path length: " << this->maxPathLength_ << " m." << endl;
		}

		// maximum obstacle size for path search
		std::vector<double> maxObstacleSizeTemp;
		if (not this->nh_.getParam("bspline_traj/max_obstacle_size", maxObstacleSizeTemp)){
			this->maxObstacleSize_  = Eigen::Vector3d (10.0, 10.0, 10.0);
			cout << "[BsplineTraj]" << ": No maximum obstacle size. Use default: [10.0, 10.0, 10.0]m." << endl;
		}
		else{
			this->maxObstacleSize_(0) = maxObstacleSizeTemp[0];
			this->maxObstacleSize_(1) = maxObstacleSizeTemp[1];
			this->maxObstacleSize_(2) = maxObstacleSizeTemp[2];
			cout << "[BsplineTraj]" << ": Maximum obstacle size: [" << maxObstacleSizeTemp[0] << ", " << maxObstacleSizeTemp[1] << ", " << maxObstacleSizeTemp[2] << "]m." << endl;
		}
	}

	void bsplineTraj::registerPub(){
		this->controlPointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/control_points", 10);
		this->currTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/trajectory", 10);
		this->astarVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/astar_path", 10);
		this->guidePointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/guide_points_and_directions", 10);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/input_trajectory", 10);
		this->inputTrajPointPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/input_trajectory_point", 10);
	}

	void bsplineTraj::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.01), &bsplineTraj::visCB, this);
	}

	void bsplineTraj::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
		this->pathSearch_.reset(new AStar);

		int maxGridX = 2 * int(this->maxObstacleSize_(0)/this->map_->getRes());
		int maxGridY = 2 * int(this->maxObstacleSize_(1)/this->map_->getRes());
		int maxGridZ = 2 * int(this->maxObstacleSize_(2)/this->map_->getRes());
		this->pathSearch_->initGridMap(map, Eigen::Vector3i(maxGridX, maxGridY, maxGridZ), this->minHeight_, this->maxHeight_);
	}

	void bsplineTraj::updateMaxVel(double maxVel){
		this->maxVel_ = maxVel;
		cout << "[BsplineTraj]: Max velocity is updated to: " << this->maxVel_ << "m/s." << endl;
	}

	void bsplineTraj::updateMaxAcc(double maxAcc){
		this->maxAcc_ = maxAcc;
		cout << "[BsplineTraj]: Max acceleration is updated to: " << this->maxAcc_ << "m/s^2" << endl;
	}

	bool bsplineTraj::inputPathCheck(const nav_msgs::Path & path, nav_msgs::Path& adjustedPath, double dt, double& finalTime){	
		if (path.poses.size() == 0) return true; // updatePath can deal with this

		std::vector<Eigen::Vector3d> curveFitPoints, adjustedCurveFitPoints;
		this->pathMsgToEigenPoints(path, curveFitPoints);
		// this->adjustPathLength(curveFitPoints, adjustedCurveFitPoints);
		this->adjustPathLengthDirect(curveFitPoints, adjustedCurveFitPoints);

		// find distance between trajectory points
		for (size_t i=0; i<adjustedCurveFitPoints.size()-1; ++i){
			double dist = (adjustedCurveFitPoints[i] - adjustedCurveFitPoints[i+1]).norm();
			if (dist > this->controlPointDistance_ * 1.5){
				return false;
			}
		}

		// remove too close points
		Eigen::Vector3d prevPoint;
		std::vector<Eigen::Vector3d> adjustedPoints;
		for (size_t i=0; i<adjustedCurveFitPoints.size(); ++i){
			Eigen::Vector3d p = adjustedCurveFitPoints[i];
			if (i == 0){
				adjustedPoints.push_back(p);
				prevPoint = p;
			}
			else{
				double dist = (p - prevPoint).norm();
				if (dist >= this->controlPointDistance_ * 0.8){
					adjustedPoints.push_back(p);
					prevPoint = p;
				}
			}
		}
		adjustedPoints.push_back(adjustedPoints.back());		

		this->eigenPointsToPathMsg(adjustedPoints, adjustedPath);
		finalTime = (adjustedCurveFitPoints.size()-1) * dt;
		return true;
	}

	bool bsplineTraj::fillPath(const nav_msgs::Path& path, nav_msgs::Path& adjustedPath){ // use linear interpotation to make input trajectory size = 4
		// if path size is less than 1, return false
		if (int(path.poses.size()) <= 1){
			return false;
		}


		if (int(path.poses.size()) == 2){
			Eigen::Vector3d ps (path.poses[0].pose.position.x, path.poses[0].pose.position.y, path.poses[0].pose.position.z);
			Eigen::Vector3d pf (path.poses[1].pose.position.x, path.poses[1].pose.position.y, path.poses[1].pose.position.z);
			Eigen::Vector3d pm1 = (pf - ps)/3.0 + ps;
			Eigen::Vector3d pm2 = 2.0 * (pf - ps)/3.0 + ps;
			geometry_msgs::PoseStamped ps1, ps2, ps3, ps4;
			ps1.pose.position.x = ps(0); ps1.pose.position.y = ps(1); ps1.pose.position.z = ps(2);
			ps2.pose.position.x = pm1(0); ps2.pose.position.y = pm1(1); ps2.pose.position.z = pm1(2);
			ps3.pose.position.x = pm2(0); ps3.pose.position.y = pm2(1); ps3.pose.position.z = pm2(2);
			ps4.pose.position.x = pf(0); ps4.pose.position.y = pf(1); ps4.pose.position.z = pf(2);
			adjustedPath.poses = {ps1, ps2, ps3, ps4};
		}


		if (int(path.poses.size()) == 3){
			Eigen::Vector3d ps (path.poses[0].pose.position.x, path.poses[0].pose.position.y, path.poses[0].pose.position.z);
			Eigen::Vector3d pm (path.poses[1].pose.position.x, path.poses[1].pose.position.y, path.poses[1].pose.position.z);
			Eigen::Vector3d pf (path.poses[2].pose.position.x, path.poses[2].pose.position.y, path.poses[2].pose.position.z);
			Eigen::Vector3d pm1 = (ps + pm)/2.0;
			Eigen::Vector3d pm2 = (pm + pf)/2.0;
			geometry_msgs::PoseStamped ps1, ps2, ps3, ps4, ps5;
			ps1.pose.position.x = ps(0); ps1.pose.position.y = ps(1); ps1.pose.position.z = ps(2);
			ps2.pose.position.x = pm1(0); ps2.pose.position.y = pm1(1); ps2.pose.position.z = pm1(2);
			ps3.pose.position.x = pm(0); ps3.pose.position.y = pm(1); ps3.pose.position.z = pm(2);
			ps4.pose.position.x = pm2(0); ps4.pose.position.y = pm2(1); ps4.pose.position.z = pm2(2);
			ps5.pose.position.x = pf(0); ps5.pose.position.y = pf(1); ps5.pose.position.z = pf(2);
			adjustedPath.poses = {ps1, ps2, ps3, ps4, ps5};			
		}


		if (int(path.poses.size()) >= 4){
			adjustedPath = path;
		}
		return true;
	}

	bool bsplineTraj::updatePath(const nav_msgs::Path& adjustedPath, const std::vector<Eigen::Vector3d>& startEndConditions){
		Eigen::Vector3d goal (adjustedPath.poses.back().pose.position.x, adjustedPath.poses.back().pose.position.y, adjustedPath.poses.back().pose.position.z);
		if (this->map_->isInflatedOccupied(goal)){
			cout << "[bsplineTraj]: Invalid goal position: " << goal.transpose() << endl;
			return false;
		}

		std::vector<Eigen::Vector3d> adjustedPathVec, inputPathVec;
		this->pathMsgToEigenPoints(adjustedPath, adjustedPathVec);
		this->adjustPathLengthDirect(adjustedPathVec, inputPathVec);
		nav_msgs::Path inputPath;
		this->eigenPointsToPathMsg(inputPathVec, inputPath);
		// nav_msgs::Path inputPath = adjustedPath;
		if (inputPath.poses.size() < 4){
			bool fillPathSuccess = this->fillPath(adjustedPath, inputPath);
			if (not fillPathSuccess){
				cout << "[bsplineTraj]: Input path point size is less (or equal) than 1." << endl;
				return false;	
			}
		} 
		this->clear();
		std::vector<Eigen::Vector3d> adjustedCurveFitPoints;
		this->pathMsgToEigenPoints(inputPath, adjustedCurveFitPoints);
		Eigen::MatrixXd controlPoints;
		this->bspline_.parameterizeToBspline(this->controlPointsTs_, adjustedCurveFitPoints, startEndConditions, controlPoints);
		this->optData_.controlPoints = controlPoints;
		int controlPointNum = controlPoints.cols();
		this->optData_.guidePoints.resize(controlPointNum);
		this->optData_.guideDirections.resize(controlPointNum);
		this->optData_.findGuidePoint.resize(controlPointNum, false);
		this->init_ = true;
		this->inputPathVis_ = adjustedCurveFitPoints;
		return true;
	}	


	void bsplineTraj::updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize){
		this->optData_.dynamicObstaclesPos = obstaclesPos;
		this->optData_.dynamicObstaclesVel = obstaclesVel;
		this->optData_.dynamicObstaclesSize = obstaclesSize;
	}


	bool bsplineTraj::makePlan(){
		// if (not this->isGoalValid()){
		// 	cout << "[BsplineTraj]: Goal is not collision free. Force Return." << endl;
		// 	return false;
		// }

		ros::Time startTime = ros::Time::now();
		// step 1. find collision segment
		this->findCollisionSeg(this->optData_.controlPoints, this->collisionSeg_); // upodate collision seg

		// step 2. A* to find collision free path
		bool pathSearchSuccess = this->pathSearch(this->collisionSeg_, this->astarPaths_);
		if (not pathSearchSuccess){
			// this->clear();
			cout << "[BsplineTraj]: Fail because of A* failure." << endl;
			return false;
		}

		// step 3. Assign guide point and directions
		this->assignGuidePointsSemiCircle(this->astarPaths_, this->collisionSeg_);

		// step 4. call solver
		bool optimizationSuccess = this->optimizeTrajectory();
		// this->clear();
		if (not optimizationSuccess){
			cout << "[BsplineTraj]: Fail because of optimizer not finding a solution." << endl; 
			return false;
		}

		// for debugging: get all costs:
		// double distanceCost, smoothnessCost, feasibilityCost, dynamicObstacleCost;
		// Eigen::MatrixXd distanceGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		// Eigen::MatrixXd smoothnessGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		// Eigen::MatrixXd feasibilityGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		// Eigen::MatrixXd dynamicObstacleGradient = Eigen::MatrixXd::Zero(3, this->optData_.controlPoints.cols());
		// this->getDistanceCost(this->optData_.controlPoints, distanceCost, distanceGradient);
		// this->getSmoothnessCost(this->optData_.controlPoints, smoothnessCost, smoothnessGradient);
		// this->getFeasibilityCost(this->optData_.controlPoints, feasibilityCost, feasibilityGradient);
		// this->getDynamicObstacleCost(this->optData_.controlPoints, dynamicObstacleCost, dynamicObstacleGradient);
		// cout << "\033[1;34m[BsplineTraj]:\033[0m" << "d: " << distanceCost << " s: " << smoothnessCost << " f: " << feasibilityCost << " do: " << dynamicObstacleCost << endl;


		// step 5. save the result to the class attribute
		this->bspline_ = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->controlPointsTs_);

		// step 6. Time reparameterization
		this->linearFeasibilityReparam();


		ros::Time endTime = ros::Time::now();
		cout << "[BsplineTraj]: Total time: " << (endTime - startTime).toSec() << endl;
		return true;
	}

	bool bsplineTraj::makePlan(nav_msgs::Path& trajectory, bool yaw){
		bool success = this->makePlan();
		trajectory = this->evalTrajToMsg(yaw);
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
		for (int i=bsplineDegree; i<=endIdx; ++i){
			// check collision of each control point
			Eigen::Vector3d p = controlPoints.col(i);
			bool hasCollision = this->map_->isInflatedOccupied(p);

			// only if collision condition changes we need to change 
			if (hasCollision != previousHasCollision){
				if (hasCollision){// if has collision and collision status changes: this means this is a start of a collision segment. record the previous point.
					pairStartIdx = i-1;
				}
				else{ // if no collision and collision status changes: this means this is an end of a collision segment. record the previous point
					pairEndIdx = i;
					std::pair<int, int> seg {pairStartIdx, pairEndIdx};
					collisionSeg.push_back(seg);
				}
			}
			
			if (hasCollision and i==endIdx-1){ // corner case
				pairEndIdx = controlPoints.cols() - 1; // goal position
				std::pair<int, int> seg {pairStartIdx, pairEndIdx};
				collisionSeg.push_back(seg);
			}

			// check the case when current collision and prev collision is false but the line collision is true
			if (i != bsplineDegree){
				if (not previousHasCollision and not hasCollision){
					if (this->map_->isInflatedOccupiedLine(controlPoints.col(i-1) , p)){
						std::pair<int, int> seg {i-1, i};
						collisionSeg.push_back(seg);
					}
				}
			}


			previousHasCollision = hasCollision;
		}
	}

	bool bsplineTraj::pathSearch(std::vector<std::pair<int, int>>& collisionSeg, std::vector<std::vector<Eigen::Vector3d>>& paths){
		paths.clear();
		std::vector<int> mergeIndices;
		int collisionSegNum = int(collisionSeg.size());
		for (int i=0; i<collisionSegNum; ++i){
			std::pair<int, int> seg = collisionSeg[i];
			Eigen::Vector3d pStart (this->optData_.controlPoints.col(seg.first));
			Eigen::Vector3d pEnd (this->optData_.controlPoints.col(seg.second));
			if (this->pathSearch_->AstarSearch(this->map_->getRes(), pStart, pEnd)){
				std::vector<Eigen::Vector3d> searchedPath = this->pathSearch_->getPath();
				searchedPath[0] = pStart;
				searchedPath.push_back(pEnd);
				paths.push_back(searchedPath);
			}
			else{
				// In some cases, because of the obstacle inflation, some control points inside the obstacle will be recognized as free.
				// In those cases, we will try merging with the next segment
				cout << "\033[1;34m[BsplineTraj]: Cannot find A* path. try merging...\033[0m" << endl;
				if (i+1 < collisionSegNum){ // in this case, it has a next segment
					std::pair<int, int> nextSeg = collisionSeg[i+1];
					if (nextSeg.first - seg.second <= 2){
						Eigen::Vector3d pStart (this->optData_.controlPoints.col(seg.first));
						Eigen::Vector3d pEnd (this->optData_.controlPoints.col(nextSeg.second));
						if (this->pathSearch_->AstarSearch(this->map_->getRes(), pStart, pEnd)){
							std::vector<Eigen::Vector3d> searchedPath = this->pathSearch_->getPath();
							searchedPath[0] = pStart;
							searchedPath.push_back(pEnd);
							paths.push_back(searchedPath);	
							mergeIndices.push_back(i);						
							++i; // jump to the next after the next
							continue;
						}
					}
					else{ 

						cout << "\033[1;34m[BsplineTraj]: Collision segment merge fail due to a big gap: \033[0m" << nextSeg.first - seg.second << endl; 
					}
				}
				else{
					cout << "\033[1;34m[BsplineTraj]: Collision segment merge fail due to insufficient collision segments.\033[0m" << endl;
				}

				cout << "[BsplineTraj]: Path Search Error. Force return. start: " << pStart.transpose() << " end: " << pEnd.transpose() << endl;
				return false; 
			}
		}	


		// if we have merged the collision segments, we will need to update in the collision seg variable
		if (int(mergeIndices.size()) != 0){
			cout << "\033[1;34m[BsplineTraj]: Merge collision segments success.\033[0m" << endl;
			int midx = 0;
			std::vector<std::pair<int, int>> collisionSegTemp;
			for (int i=0; i<collisionSegNum; ++i){
				if (midx < int(mergeIndices.size()) and i == mergeIndices[midx]){
					collisionSegTemp.push_back(std::pair<int, int> {collisionSeg[i].first, collisionSeg[i+1].second});
					++i;
					++midx;
				}
				else{
					collisionSeg.push_back(collisionSeg[i]);
				}
			}
			collisionSeg = collisionSegTemp;
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
			for (int controlPointIdx=seg.first+1; controlPointIdx<seg.second; ++controlPointIdx){ // iterate through all collision control points
				bool findGuidePoint = this->findGuidePointSemiCircle(controlPointIdx, seg, path, guidePoint);
				this->optData_.guidePoints[controlPointIdx].push_back(guidePoint);
				guideDirection = (guidePoint - this->optData_.controlPoints.col(controlPointIdx))/(guidePoint - this->optData_.controlPoints.col(controlPointIdx)).norm();
				// guideDirection = (guidePoint - this->optData_.controlPoints.col(controlPointIdx));
				this->optData_.guideDirections[controlPointIdx].push_back(guideDirection);
				if (not findGuidePoint){
					ROS_ERROR("[BsplineTraj]: Impossible Assignment. Something wrong!");
				}
			}

			bool lineCollision = (seg.second - seg.first - 1 == 0);
			if (lineCollision){
				int forwardIdx = 1;
				this->findGuidePointSemiCircle(seg.first, seg, path, guidePoint);
				Eigen::Vector3d midPoint = (this->optData_.controlPoints.col(seg.first) + this->optData_.controlPoints.col(seg.second))/2.0;
				guideDirection = (guidePoint - midPoint)/(guidePoint - midPoint).norm();
				for (int controlPointIdx=seg.first-forwardIdx; controlPointIdx<=seg.second+forwardIdx; ++controlPointIdx){
					if (controlPointIdx >= bsplineDegree and controlPointIdx <= int((this->optData_.controlPoints.cols() - bsplineDegree - 1))){
						this->optData_.guidePoints[controlPointIdx].push_back(guidePoint);
						this->optData_.guideDirections[controlPointIdx].push_back(guideDirection);
					}
				}


				// for (int controlPointIdx=seg.first; controlPointIdx<=seg.second; ++controlPointIdx){ // iterate through all collision control points
				// 	bool findGuidePoint = this->findGuidePointSemiCircle(controlPointIdx, seg, path, guidePoint);

				// 	cout << "find guide point: " << findGuidePoint << " : " << guidePoint.transpose() << endl;
				// 	this->optData_.guidePoints[controlPointIdx].push_back(guidePoint);
				// 	Eigen::Vector3d midPoint = (this->optData_.controlPoints.col(seg.first) + this->optData_.controlPoints.col(seg.second))/2.0;
				// 	guideDirection = (guidePoint - midPoint)/(guidePoint - midPoint).norm();
				// 	cout << "guide direction is: " << guideDirection.transpose() << endl;
				// 	// guideDirection = (guidePoint - this->optData_.controlPoints.col(controlPointIdx))/(guidePoint - this->optData_.controlPoints.col(controlPointIdx)).norm();
				// 	// guideDirection = (guidePoint - this->optData_.controlPoints.col(controlPointIdx));
				// 	this->optData_.guideDirections[controlPointIdx].push_back(guideDirection);
				// 	if (not findGuidePoint){
				// 		ROS_ERROR("[BsplineTraj]: Impossible Assignment. Something wrong!");
				// 	}
				// }
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

		// for overlapped collision points: check its distance to current guide point: if larger than threshold, it means we need to reguide and find its corresponding segment
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
		this->optimize();
		double weightDistance0 = this->weightDistance_;
		double weightDynamicObstacle0 = this->weightDynamicObstacle_;
		int failCount = 0; 
		std::vector<vector<Eigen::Vector3d>> tempAstarPaths; // in case path search fail
		bool hasCollision, hasDynamicCollision;
		ros::Time startTime = ros::Time::now();
		while (ros::ok()){
			hasCollision = this->hasCollisionTrajectory(this->optData_.controlPoints);
			if (this->optData_.dynamicObstaclesPos.size() != 0){
				hasDynamicCollision = this->hasDynamicCollisionTrajectory(this->optData_.controlPoints);
			}
			else{
				hasDynamicCollision = false;
			}

			if (not hasCollision and not hasDynamicCollision){
				break;
			}

			ros::Time currTime = ros::Time::now();
			if ((currTime - startTime).toSec() > 0.03){
				this->weightDistance_ = weightDistance0;
				this->weightDynamicObstacle_ = weightDynamicObstacle0;
				cout << "[BsplineTraj]: Optimization timeout." << endl;
				return false;			
			}

			if (failCount >= 4){
				std::vector<std::pair<int, int>> collisionSeg;
				this->findCollisionSeg(this->optData_.controlPoints, collisionSeg);
				bool pathSearchSuccess = this->pathSearch(collisionSeg, tempAstarPaths);	
				if (pathSearchSuccess){
					this->astarPaths_ = tempAstarPaths; 
					this->assignGuidePointsSemiCircle(tempAstarPaths, collisionSeg);
				}				
			}

			if (failCount >= 8){
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
			}
			this->optimize();
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

	void bsplineTraj::adjustPathLength(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& adjustedPath){
		// iterate through the raw path
		double totalLength = 0.0;
		bool free = false;
		bool exceedLength = false;
		double minLength = 0.0;
		for (size_t i=0; i<path.size()-1; ++i){
			Eigen::Vector3d p1 = path[i];
			Eigen::Vector3d p2 = path[i+1];
			totalLength += (p2 - p1).norm();
			if (totalLength >= this->maxPathLength_){
				exceedLength = true;
			}
			adjustedPath.push_back(p1);

			if (exceedLength){
				free = not this->map_->isInflatedOccupiedLine(p1, p2);
				if (free and minLength >= 1.5){
					adjustedPath.push_back(p2);
					return;
				}
			}

			bool occupied = this->map_->isInflatedOccupiedLine(p1, p2);
			if (occupied){
				minLength = 0.0;
			}
			else{
				minLength += (p2 - p1).norm();
			}
		}
		adjustedPath.push_back(path.back());
	}

	void bsplineTraj::adjustPathLengthDirect(const std::vector<Eigen::Vector3d>& path, std::vector<Eigen::Vector3d>& adjustedPath){
		// create a variable to record the previous goal distance 
		static double prevPathLength = 0.0; // although I say the path length, but actually is the goal distance
		// cout << "\033[1;34m[BsplineTraj]: prev path length is:\033[0m " << prevPathLength << endl;

		// iterate through the raw path
		double totalLength = 0.0;
		bool free = false;
		bool exceedLength = false;
		double minLength = 0.0;
		Eigen::Vector3d pStart = path[0];
		for (size_t i=0; i<path.size()-1; ++i){
			Eigen::Vector3d p1 = path[i];
			Eigen::Vector3d p2 = path[i+1];
			totalLength = (p2 - pStart).norm();
			if (totalLength >= std::max(prevPathLength, this->maxPathLength_)){
				exceedLength = true;
			}
			adjustedPath.push_back(p1);

			if (exceedLength){
				free = not this->map_->isInflatedOccupiedLine(p1, p2);
				if (free and minLength >= 1.5){
					adjustedPath.push_back(p2);
					prevPathLength = totalLength;
					return;
				}
			}

			bool occupied = this->map_->isInflatedOccupiedLine(p1, p2);
			if (occupied){
				minLength = 0.0;
			}
			else{
				minLength += (p2 - p1).norm();
			}
		}
		adjustedPath.push_back(path.back());	
		prevPathLength = totalLength;	
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
		// total cost and gradient (because feasibility includes both velocity and acc, so divide 2 for scaling)
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
		double costTemp;
		Eigen::Vector3d gradientTemp;
		double a, b, c;
		a = 3.0 * this->dthresh_; b = -3.0 * pow(this->dthresh_, 2); c = pow(this->dthresh_, 3);
		const double heightDistThresh = 0.2;
		double ah = 3.0 * heightDistThresh, bh = -3 * pow(heightDistThresh, 2), ch = pow(heightDistThresh, 3);
		for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){ // for each control points
			for (size_t j=0; j<this->optData_.guidePoints[i].size(); ++j){ // each control points we check all the guide points and guide directions
				double dist = (controlPoints.col(i) - this->optData_.guidePoints[i][j]).dot(this->optData_.guideDirections[i][j]);
				bool unknownGuidePoint = this->map_->isUnknown(this->optData_.guidePoints[i][j]);

				double distErr = this->dthresh_ - dist;

				Eigen::Vector3d grad = this->optData_.guideDirections[i][j];
				// Eigen::Vector3d grad = (this->optData_.guidePoints[i][j] - controlPoints.col(i))/(this->optData_.guidePoints[i][j] - controlPoints.col(i)).norm();
				// grad(2) = 0;

				// if (distErr <= 0){
				// 	// no punishment	
				// }
				if (distErr <= - 1.0 * this->dthresh_){
					// punishment for going too far
					costTemp = pow(-distErr, 3);
					gradientTemp = 3.0 * pow(-distErr, 2) * grad;
					if (not this->planInZAxis_){
						gradientTemp(2) = 0.0;
					}
					cost += costTemp;
					gradient.col(i) += gradientTemp;
				}
				else if (distErr > 0 and distErr <= this->dthresh_){
					// cost += pow(distErr, 3);
					// gradient.col(i) += -3.0 * pow(distErr, 2) * grad;
					costTemp = pow(distErr, 3);
					gradientTemp = -3.0 * pow(distErr, 2) * grad;
	
					if (unknownGuidePoint){
						costTemp *= this->uncertainAwareFactor_;
						gradientTemp *= this->uncertainAwareFactor_;
					}
					if (not this->planInZAxis_){
						gradientTemp(2) = 0.0;
					}

					cost += costTemp;
					gradient.col(i) += gradientTemp;
				}
				else if (distErr >= this->dthresh_){
					// cost += a * pow(distErr, 2) + b * distErr + c;
					// gradient.col(i) += -(2 * a * distErr + b) * grad;
					costTemp = a * pow(distErr, 2) + b * distErr + c;
					gradientTemp = -(2 * a * distErr + b) * grad;
					
					if (unknownGuidePoint){
						costTemp *= this->uncertainAwareFactor_;
						gradientTemp *= this->uncertainAwareFactor_;
					}
					if (not this->planInZAxis_){
						gradientTemp(2) = 0.0;
					}
					cost += costTemp;
					gradient.col(i) += gradientTemp;
				}
			}

			if (this->planInZAxis_){
				// minimum and maximum height cost 
				double heightDistMin = controlPoints.col(i)(2) - this->minHeight_;
				double heightDistMax = controlPoints.col(i)(2) - this->maxHeight_;
				if (heightDistMin < 0){
					double distErr = heightDistThresh - heightDistMin;
					cost += ah * pow(distErr, 2) + bh * distErr + ch;
					gradient.col(i) += -(2 * ah * distErr + bh) * Eigen::Vector3d (-1.0, 0.0, 0.0);

				}
				else if (heightDistMin >= 0 and heightDistMax < heightDistThresh){
					double distErr = heightDistThresh - heightDistMin;
					cost += pow(distErr, 3);
					gradient.col(i) += -3.0 * pow(distErr, 2) * Eigen::Vector3d (-1.0, 0.0, 0.0);
				}
				else if (heightDistMin >= heightDistThresh){
					// no punishment
				}


				if (heightDistMax > 0){
					double distErr = heightDistThresh + heightDistMax;
					cost += ah * pow(distErr, 2) + bh * distErr + ch;
					gradient.col(i) += -(2 * ah * distErr + bh) * Eigen::Vector3d (1.0, 0.0, 0.0);
				}
				else if (heightDistMax <=0 and heightDistMax >= -heightDistThresh){
					double distErr = heightDistThresh + heightDistMax;
					cost += pow(distErr, 3);
					gradient.col(i) += -3.0 * pow(distErr, 2) * Eigen::Vector3d (1.0, 0.0, 0.0);
				} 
				else if (heightDistMax < -heightDistThresh){
					// no punishment
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
		// cost /= controlPoints.cols();
		// gradient /= controlPoints.cols();
	}

	void bsplineTraj::getFeasibilityCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		// velocity and acceleration cost
		cost = 0.0;
		double maxVel = 1.0;
		double maxAcc = 1.0;

		// velocity cost
		double tsInvSqr = 1/pow(this->controlPointsTs_, 2); // for balancing velocity and acceleration scales
		Eigen::Vector3d vi;
		for (int i=0; i<controlPoints.cols()-1; ++i){
			vi = (controlPoints.col(i+1) - controlPoints.col(i))/this->controlPointsTs_;
			for (int j=0; j<3; ++j){ // 3 axis
				if (vi(j) > maxVel){
					cost += pow(vi(j) - maxVel, 2) * tsInvSqr;
					gradient(j, i) += -2 * (vi(j) - maxVel)/(this->controlPointsTs_) * tsInvSqr;
					gradient(j, i+1) += 2 * (vi(j) - maxVel)/(this->controlPointsTs_) * tsInvSqr;
				}
				else if (vi(j) < -maxVel){
					cost += pow(vi(j) + maxVel, 2) * tsInvSqr;
					gradient(j, i) += -2 * (vi(j) + maxVel)/(this->controlPointsTs_) * tsInvSqr;
					gradient(j, i+1) += 2 * (vi(j) + maxVel)/(this->controlPointsTs_) * tsInvSqr;
				}				
			}
		}

		// acceleration cost
		Eigen::Vector3d ai;
		for (int i=0; i<controlPoints.cols()-2; ++i){
			ai = (controlPoints.col(i+2) - 2 * controlPoints.col(i+1) + controlPoints.col(i)) * tsInvSqr;
			for (int j=0; j<3; ++j){
				if (ai(j) > maxAcc){
					cost += pow(ai(j) - maxAcc, 2);
					gradient(j, i) += 2 * (ai(j) - maxAcc) * tsInvSqr;
					gradient(j, i+1) += -4 * (ai(j) - maxAcc) * tsInvSqr;
					gradient(j, i+2) += 2 * (ai(j) - maxAcc) * tsInvSqr;
				}
				else if (ai(j) < -maxAcc){
					cost += pow(ai(j) + maxAcc, 2);
					gradient(j, i) += 2 * (ai(j) + maxAcc) * tsInvSqr;
					gradient(j, i+1) += -4 * (ai(j) + maxAcc) * tsInvSqr;
					gradient(j, i+2) += 2 * (ai(j) + maxAcc) * tsInvSqr;
				}
			}
		}

		// cost /= controlPoints.cols();
		// gradient /= controlPoints.cols();
	}

	void bsplineTraj::getDynamicObstacleCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
		cost = 0;
		if (this->optData_.dynamicObstaclesPos.size() == 0) return;

		// iterate through each control points
		const int skipFactor = 2.0;
		int predictionNum = int(this->predHorizon_/this->ts_);
		double a, b, c;
		a = 3.0 * this->distThreshDynamic_; b = -3 * pow(this->distThreshDynamic_, 2); c = pow(this->distThreshDynamic_, 3);
		for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){
			Eigen::Vector3d controlPoint = controlPoints.col(i);
			for (size_t j=0; j<this->optData_.dynamicObstaclesPos.size(); ++j){
				double size = pow(pow(this->optData_.dynamicObstaclesSize[j](0)/2, 2) + pow(this->optData_.dynamicObstaclesSize[j](1)/2, 2), 0.5);
				// double size = std::max(this->optData_.dynamicObstaclesSize[j](0)/2, this->optData_.dynamicObstaclesSize[j](1)/2);
				// double size = std::min(this->optData_.dynamicObstaclesSize[j](0)/2, this->optData_.dynamicObstaclesSize[j](1)/2);
				Eigen::Vector3d obstacleVel = this->optData_.dynamicObstaclesVel[j];

				for (int n=0; n<=predictionNum; n+=skipFactor){
					Eigen::Vector3d obstaclesPos = this->optData_.dynamicObstaclesPos[j] + double(n * this->ts_) * obstacleVel; // predicted obstacle state
					double distThresh = (1 - double(n/predictionNum) * 0.2) * this->distThreshDynamic_; // linearly decrease to half
					Eigen::Vector3d diff = controlPoint - obstaclesPos;
					diff(2) = 0.0; // ignore z difference
					double dist = diff.norm() - size; // actual distance to obstacle (need to minus obstacle size)
					double distErr = distThresh - dist;
					Eigen::Vector3d grad = diff/diff.norm();


					if (distErr <= 0){
						// no punishment	
					}
					else if (distErr > 0 and distErr <= distThresh){
						cost += pow(distErr, 3);
						gradient.col(i) += -3.0 * pow(distErr, 2) * grad;
					}
					else if (distErr >= distThresh){
						cost += (a * pow(distErr, 2) + b * distErr + c);
						gradient.col(i) += -(2 * a * distErr + b) * grad;
					}
				}


				// Eigen::Vector3d obstaclesPos = this->optData_.dynamicObstaclesPos[j];
				// Eigen::Vector3d diff = controlPoint - obstaclesPos;
				// diff(2) = 0.0; // ignore z difference
				// double dist = diff.norm() - size; // actual distance to obstacle (need to minus obstacle size)
				// double distThresh = this->distThreshDynamic_;
				// double distErr = distThresh - dist;
				// Eigen::Vector3d grad = 2.0 * diff/diff.norm();


				// if (distErr <= 0){
				// 	// no punishment	
				// }
				// else if (distErr > 0 and distErr <= distThresh){
				// 	cost += pow(distErr, 3);
				// 	gradient.col(i) += -3.0 * pow(distErr, 2) * grad;
				// }
				// else if (distErr >= distThresh){
				// 	cost += a * pow(distErr, 2) + b * distErr + c;
				// 	gradient.col(i) += -(2 * a * distErr + b) * grad;
				// }
			}
		}
	}

	// void bsplineTraj::getDynamicObstacleCost(const Eigen::MatrixXd& controlPoints, double& cost, Eigen::MatrixXd& gradient){
	// 	cost = 0;
	// 	if (this->optData_.dynamicObstaclesPos.size() == 0) return;

	// 	// iterate through each control points
	// 	int predictionNum = int(this->predHorizon_/this->ts_);
	// 	double a, b, c;
	// 	a = 3.0 * this->distThreshDynamic_; b = -3 * pow(this->distThreshDynamic_, 2); c = pow(this->distThreshDynamic_, 3);
	// 	for (int i=bsplineDegree; i<=controlPoints.cols()-bsplineDegree-1; ++i){
	// 		Eigen::Vector3d controlPoint = controlPoints.col(i);
	// 		for (size_t j=0; j<this->optData_.dynamicObstaclesPos.size(); ++j){
	// 			// double size = pow(pow(this->optData_.dynamicObstaclesSize[j](0)/2, 2) + pow(this->optData_.dynamicObstaclesSize[j](1), 2)/2, 0.5);
	// 			double size = std::min(this->optData_.dynamicObstaclesSize[j](0)/2, this->optData_.dynamicObstaclesSize[j](1)/2);
	// 			Eigen::Vector3d obstaclePos = this->optData_.dynamicObstaclesPos[j];
	// 			Eigen::Vector3d obstacleVel = this->optData_.dynamicObstaclesVel[j];

	// 			double radius = size+this->distThreshDynamic_;
	// 			// int status =2;
	// 			int status = this->isInDistanceField(obstaclePos, obstacleVel, controlPoint, radius);
	// 			double distErr;
	// 			Eigen::Vector3d grad;
	// 			if (status == 1){ // in the polygon region
	// 				this->getDynamicCostAndGradPolygon(obstaclePos, obstacleVel, controlPoint, radius, distErr, grad);
	// 			}

	// 			if (status == 2){ // in the circle region
	// 				this->getDynamicCostAndGradCircle(obstaclePos, controlPoint, radius, distErr, grad);
	// 			}

	// 			if (status != 0){
	// 				// cost += pow(distErr, 2);
	// 				// gradient.col(i) += 2.0 * pow(distErr, 1) * grad;
	// 				if (distErr <= 0){
	// 					// no punishment	
	// 				}
	// 				else if (distErr > 0 and distErr <= this->distThreshDynamic_){
	// 					cost += pow(distErr, 3);
	// 					gradient.col(i) += 3.0 * pow(distErr, 2) * grad;
	// 				}
	// 				else if (distErr >= this->distThreshDynamic_){
	// 					cost += a * pow(distErr, 2) + b * distErr + c;
	// 					gradient.col(i) += (2 * a * distErr + b) * grad;
	// 				}
	// 			}
	// 		}


	// 	}
	// }

	void bsplineTraj::linearFeasibilityReparam(){
		// find the maximum velocity and acceleration
		double trajMaxVel = 0.0; 
		double trajMaxAcc = 0.0; 
		trajPlanner::bspline trajVel = this->bspline_.getDerivative();
		trajPlanner::bspline trajAcc = trajVel.getDerivative();
		for (double t=0.0; t<this->bspline_.getDuration(); t+=this->ts_){
			Eigen::Vector3d vel = trajVel.at(t);
			Eigen::Vector3d acc = trajAcc.at(t);
			if (vel.norm() > trajMaxVel){
				trajMaxVel = vel.norm();
			}

			if (acc.norm() > trajMaxAcc){
				trajMaxAcc = acc.norm();
			}
		}

		double factorVel = this->maxVel_/trajMaxVel;
		double factorAcc = sqrt(this->maxAcc_/trajMaxAcc);
		this->linearFactor_ = std::min(factorVel, factorAcc);
	}

	double bsplineTraj::getLinearReparamTime(double t){
		return this->linearFactor_ * t;
	}

	double bsplineTraj::getLinearFactor(){
		return this->linearFactor_;
	}

	void bsplineTraj::visCB(const ros::TimerEvent&){
		if (this->init_){
			this->publishControlPoints();
			this->publishCurrTraj();
			this->publishAstarPath();
			this->publishGuidePoints();
			this->publishInputTraj();
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
			point.lifetime = ros::Duration(0.1);
			point.scale.x = 0.3;
			point.scale.y = 0.3;
			point.scale.z = 0.3;
			point.color.a = 1.0;
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
		std::vector<std::vector<Eigen::Vector3d>> astarPathsSC;
		// this->shortcutPaths(this->astarPaths_, astarPathsSC);
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
				point.scale.x = 0.05;
				point.scale.y = 0.05;
				point.scale.z = 0.05;
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
		for (int i=0; i<this->optData_.controlPoints.cols(); ++i){
			for (size_t j=0; j<this->optData_.guidePoints[i].size(); ++j){
				visualization_msgs::Marker pointG;
				Eigen::Vector3d p = this->optData_.guidePoints[i][j]; 
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


		// guide directions
		int numGuideDirections = 0;
		for (int i=0; i<this->optData_.controlPoints.cols(); ++i){
			for (size_t j=0; j<this->optData_.guidePoints[i].size(); ++j){
				visualization_msgs::Marker arrow;
				Eigen::Vector3d pGuide = this->optData_.guidePoints[i][j]; 
				Eigen::Vector3d p = this->optData_.controlPoints.col(i); 
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


		msg.markers = msgVec;
		this->guidePointsVisPub_.publish(msg);
	}


	void bsplineTraj::publishInputTraj(){
		if (this->inputPathVis_.size() != 0){
			nav_msgs::Path inputTrajPathMsg;
			this->eigenPointsToPathMsg(this->inputPathVis_, inputTrajPathMsg);
			visualization_msgs::MarkerArray inputPathPoints;

			for (size_t i=0; i<this->inputPathVis_.size(); ++i){
				Eigen::Vector3d p = this->inputPathVis_[i];
				visualization_msgs::Marker point;
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "input_trajectory_point";
				point.id = i;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = p(0);
				point.pose.position.y = p(1);
				point.pose.position.z = p(2);
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.2;
				point.scale.y = 0.2;
				point.scale.z = 0.2;
				point.color.a = 1.0;
				point.color.r = 0.0;
				point.color.g = 1.0;
				point.color.b = 0.0;
				inputPathPoints.markers.push_back(point);
			}


			this->inputTrajPointPub_.publish(inputPathPoints);
			this->inputTrajPub_.publish(inputTrajPathMsg);
		}
	}

	double bsplineTraj::getInitTs(){
		return this->controlPointDistance_/this->maxVel_;
		// return this->controlPointsTs_;
	}

	double bsplineTraj::getControlPointTs(){
		return this->controlPointsTs_;
	}

	double bsplineTraj::getControlPointDist(){
		return this->controlPointDistance_;
	}

	trajPlanner::bspline bsplineTraj::getTrajectory(){
		return this->bspline_;
	}

	geometry_msgs::PoseStamped bsplineTraj::getPose(double t, bool yaw){
		geometry_msgs::PoseStamped ps;
		Eigen::Vector3d p = this->bspline_.at(t);

		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);

		if (yaw){
			// orientation:
			trajPlanner::bspline velBspline = this->bspline_.getDerivative();
			Eigen::Vector3d vel = velBspline.at(t);
			ps.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, atan2(vel(1), vel(0)));
		}
		return ps;
	}

	double bsplineTraj::getDuration(){
		return this->bspline_.getDuration();
	}

	double bsplineTraj::getTimestep(){
		return this->ts_;
	}

	Eigen::MatrixXd bsplineTraj::getControlPoints(){
		return this->optData_.controlPoints;
	}

	std::vector<Eigen::Vector3d> bsplineTraj::evalTraj(){
		double ts = this->map_->getRes()/(this->maxVel_)/2.0;
		return this->evalTraj(ts);
	}	

	std::vector<Eigen::Vector3d> bsplineTraj::evalTraj(double dt){
		std::vector<Eigen::Vector3d> traj;
		Eigen::Vector3d p;
		trajPlanner::bspline bsplineTraj = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->controlPointsTs_);
		for (double t=0; t<=bsplineTraj.getDuration(); t+=dt){
			p = bsplineTraj.at(t);
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

	void bsplineTraj::writeCurrentTrajInfo(const std::string& filePath, double dt){
		std::string velPath = filePath + "/vel_info.txt";
		std::string accPath = filePath + "/acc_info.txt";
		std::string velAdjustedPath = filePath + "/vel_adjusted_info.txt";
		std::string accAdjustedPath = filePath + "/acc_adjusted_info.txt";
		std::ofstream velInfo (velPath);
		std::ofstream accInfo (accPath);
		std::ofstream velAdjustedInfo (velAdjustedPath);
		std::ofstream accAdjustedInfo (accAdjustedPath);

		trajPlanner::bspline velBspline = this->bspline_.getDerivative();
		trajPlanner::bspline accBspline = velBspline.getDerivative();
		for (double t=0.0; t<=this->bspline_.getDuration(); t+=dt){
			Eigen::Vector3d vel = velBspline.at(t);
			Eigen::Vector3d acc = accBspline.at(t);
			velInfo << t << " " << vel(0) << " " << vel(1) << " " << vel(2) << endl;
			accInfo << t << " " << acc(0) << " " << acc(1) << " " << acc(2) << endl;

		}

		double linearReparamFactor = this->getLinearFactor();
		for (double t=0.0; this->getLinearReparamTime(t)<=this->bspline_.getDuration(); t+=dt){
			double reparamTime = this->getLinearReparamTime(t);
			Eigen::Vector3d velAdjusted = velBspline.at(reparamTime) * linearReparamFactor;
			Eigen::Vector3d accAdjusted = velBspline.at(reparamTime) * pow(linearReparamFactor, 2);
			velAdjustedInfo << t << " " << velAdjusted(0) << " " << velAdjusted(1) << " " << velAdjusted(2) << endl;
			accAdjustedInfo << t << " " << accAdjusted(0) << " " << accAdjusted(1) << " " << accAdjusted(2) << endl;			
		}
		velInfo.close();
		accInfo.close();
		velAdjustedInfo.close();
		accAdjustedInfo.close();
	}

	nav_msgs::Path bsplineTraj::evalTrajToMsg(bool yaw){
		return this->evalTrajToMsg(this->ts_, yaw);
	}

	nav_msgs::Path bsplineTraj::evalTrajToMsg(double dt, bool yaw){
		std::vector<Eigen::Vector3d> trajTemp = this->evalTraj(dt);
		nav_msgs::Path traj;
		this->eigenPointsToPathMsg(trajTemp, traj);
		trajPlanner::bspline velBspline = this->bspline_.getDerivative();
		size_t i = 0;
		while (i < traj.poses.size()){
			double t = (double) i * dt;
			Eigen::Vector3d vel = velBspline.at(t);
			if (yaw){
				traj.poses[i].pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, atan2(vel(1), vel(0)));
			}
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