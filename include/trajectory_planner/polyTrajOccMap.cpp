/*
	FILE: polyTrajOccMap.cpp
	---------------------------
	minimum snap polynomial trajectory planner implementation based on occupancy grid map
*/

#include <trajectory_planner/polyTrajOccMap.h>

namespace trajPlanner{
	polyTrajOccMap::polyTrajOccMap(const ros::NodeHandle& nh) : nh_(nh){
		this->initParam();
		this->registerPub();
		this->registerCallback();
		if (this->usePWL_){
			this->initPWLSolver();
		}
		this->setDefaultInit(); // vel acc (this can be overwritten by user defined initial condition)
	}

	void polyTrajOccMap::initParam(){
		// Polynomial Degree
		if (not this->nh_.getParam("poly_traj/polynomial_degree", this->polyDegree_)){
			this->polyDegree_ = 7;
			cout << "[Trajectory Planner INFO]: No Polynomail Degree Parameter. Use default: 7." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Polynomial Degree: " << this->polyDegree_ << endl;
		}

		// Differential Degree: (3: Min. Jerk, 4: Min. Snap)
		if (not this->nh_.getParam("poly_traj/differential_degree", this->diffDegree_)){
			this->diffDegree_ = 4;
			cout << "[Trajectory Planner INFO]: No Differential Degree Parameter. Use default: 4." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Differential Degree: " << this->diffDegree_ << endl;
		}

		// Continuity Degree
		if (not this->nh_.getParam("poly_traj/continuity_degree", this->continuityDegree_)){
			this->continuityDegree_ = 4;
			cout << "[Trajectory Planner INFO]: No Continuity Degree Parameter. Use default: 4." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Continuity Degree: " << this->continuityDegree_ << endl;
		}

		// Desired Velocity
		if (not this->nh_.getParam("poly_traj/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[Trajectory Planner INFO]: No Desired Velocity Parameter. Use default: 1.0 m/s." << endl;
		}	
		else{
			cout << "[Trajectory Planner INFO]: Desired Velocity: " << this->desiredVel_ << " m/s." << endl;
		}

		if (not this->nh_.getParam("poly_traj/initial_radius", this->initR_)){
			this->initR_ = 0.5;
			cout << "[Trajectory Planner INFO]: No Initial Corridor Radius Parameter. Use default: 0.5 m" << endl;
		} 
		else{
			cout << "[Trajectory Planner INFO]: Corridor Radius: " << this->initR_ << " m." << endl;
		}

		// Max Solving Time (Iteratively generating collision free trajectory)
		if (not this->nh_.getParam("poly_traj/timeout", this->timeout_)){
			this->timeout_ = 0.1;
			cout << "[Trajectory Planner INFO]: No Timeout Parameter. Use default: 0.1." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Time out is set to: " << this->timeout_ << " s." << endl;
		}

		// Corridor Resolution
		if (not this->nh_.getParam("poly_traj/corridor_res", this->corridorRes_)){
			this->corridorRes_ = 5.0;
			cout << "[Trajectory Planenr INFO]: No Corridor Resolution Parameter. Use default: 5.0." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Corridor Resolution: " << this->corridorRes_ << endl;
		}

		// Factor of shrinking (Corridor size)
		if (not this->nh_.getParam("poly_traj/shrinking_factor", this->fs_)){
			this->fs_ = 0.8;
			cout << "[Trajectory Planner INFO]: No Shrinking Factor Parameter. Use default: 0.8." << endl; 
		}
		else{
			cout << "[Trajectory Planner INFO]: Shrinking Factor: " << this->fs_ << endl;
		}

		// use soft constraint or not
		if (not this->nh_.getParam("poly_traj/soft_constraint", this->softConstraint_)){
			this->softConstraint_ = false;
			cout << "[Trajectory Planner INFO]: No Soft Constraint Parameter. Use default: false." << endl;
		} 


		if (this->softConstraint_){
			if (not this->nh_.getParam("poly_traj/constraint_radius", this->softConstraintRadius_)){
				this->softConstraintRadius_ = 0.5;
				cout << "[Trajectory Planner INFO]: No Soft Constraint Radius Parameter. Use default: 0.5 m." << endl;
			}
		}

		// delta T: for collision checking
		if (not this->nh_.getParam("poly_traj/sample_delta_time", this->delT_)){
			this->delT_ = 0.1;
			cout << "[Trajectory Planner INFO]: No Sample Time Parameter. Use default: 0.1." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Sample Time: " << this->delT_ << endl;
		}

		// Max Iteration (Iteratively generating collision free trajectory)
		if (not this->nh_.getParam("poly_traj/maximum_iteration_num", this->maxIter_)){
			this->maxIter_ = 20;
			cout << "[Trajectory Planner INFO]: No Maximum Iteration Parameter. Use default: 20." << endl;
		}
		else{
			cout << "[Trajectory Planner INFO]: Maximum Iteration Number: " << this->maxIter_ << endl;
		}

		// use pwl traj as failsafe
		if (not this->nh_.getParam("poly_traj/use_pwl_failsafe", this->usePWL_)){
			this->usePWL_ = false;
			cout << "[Trajectory Planner INFO]: No use pwl option. Use default: not use." << endl;
		}
 	}

	void polyTrajOccMap::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void polyTrajOccMap::registerPub(){
		this->trajVisPub_ = this->nh_.advertise<nav_msgs::Path>("poly_traj/tajectory", 10);
	}

	void polyTrajOccMap::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &polyTrajOccMap::visCB, this);
	}

	void polyTrajOccMap::initSolver(){
		this->trajSolver_.reset(new trajPlanner::polyTrajSolver (this->polyDegree_, this->diffDegree_, this->continuityDegree_, this->desiredVel_));
	}

	void polyTrajOccMap::initPWLSolver(){
		this->pwlTrajSolver_.reset(new trajPlanner::pwlTraj (this->nh_));
	}

	void polyTrajOccMap::updatePath(const nav_msgs::Path& path){
		std::vector<trajPlanner::pose> trajPath;
		for (geometry_msgs::PoseStamped p : path.poses){
			trajPlanner::pose trajP (p.pose.position.x, p.pose.position.y, p.pose.position.z);
			trajPath.push_back(trajP);
		}
		this->updatePath(trajPath);
	}

	void polyTrajOccMap::updatePath(const std::vector<pose>& path){
		this->path_ = path;
	}

	void polyTrajOccMap::updateInitVel(double vx, double vy, double vz){
		geometry_msgs::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateInitVel(v);
	}

	void polyTrajOccMap::updateInitVel(const geometry_msgs::Twist& v){
		this->initVel_ = v;
	}

	void polyTrajOccMap::updateInitAcc(double ax, double ay, double az){
		geometry_msgs::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateInitAcc(a);
	}


	void polyTrajOccMap::updateInitAcc(const geometry_msgs::Twist& a){
		this->initAcc_ = a;
	}

	void polyTrajOccMap::setDefaultInit(){
		// initialize 
		this->updateInitVel(0, 0, 0);
		this->updateInitAcc(0, 0, 0); 
	}

	void polyTrajOccMap::makePlan(std::vector<pose>& trajectory){
		this->findValidTraj_ = false;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->findValidTraj_ = true;
			return;
		}

		this->initSolver(); // polynomial solver
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec (this->path_.size()-1, corridorSize);

		int countIter = 0;
		bool valid = false;
		ros::Time startTime = ros::Time::now();
		ros::Time endTime;
		while (ros::ok() and not valid){
			endTime = ros::Time::now();
			if ((endTime - startTime).toSec() >= this->timeout_){
				cout << "[Trajectory Planner INFO]: Timeout!" << endl;
				break;
			}
			this->trajSolver_->setCorridorConstraint(corridorSizeVec, this->corridorRes_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraint_, this->softConstraint_, 0); // no z direction soft constraint
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, this->delT_);

			std::set<int> collisionSeg;
			valid = not this->checkCollisionTraj(trajectory, this->delT_, collisionSeg);

			if (not valid){
				this->adjustCorridorSize(collisionSeg, corridorSizeVec);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}


		if (valid){
			// cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			// cout << "[Trajectory Planner INFO]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->usePWL_){
				this->pwlTrajSolver_->updatePath(this->path_);
				this->pwlTrajSolver_->makePlan(trajectory, this->delT_);
			}
		}
		this->trajMsgConverter(trajectory, this->trajVisMsg_);
	}

	void polyTrajOccMap::makePlan(nav_msgs::Path& trajectory){
		std::vector<pose> trajTemp;
		this->makePlan(trajTemp);
		this->trajMsgConverter(trajTemp, trajectory);
	}

	void polyTrajOccMap::visCB(const ros::TimerEvent&){
		this->publishTrajVis();
	}

	void polyTrajOccMap::publishTrajVis(){
		this->trajVisPub_.publish(this->trajVisMsg_);
	}

	geometry_msgs::PoseStamped polyTrajOccMap::getPose(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}

		if (this->usePWL_){
			geometry_msgs::PoseStamped ps;
			if (this->findValidTraj_){
				trajPlanner::pose p = this->trajSolver_->getPose(t);
				ps.pose.position.x = p.x;
				ps.pose.position.y = p.y;
				ps.pose.position.z = p.z;
				geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, p.yaw);
				ps.pose.orientation = quat;
				ps.header.stamp = ros::Time();
				ps.header.frame_id = "map";
			}
			else{
				ps = this->pwlTrajSolver_->getPose(t);
			}
		}
		
		geometry_msgs::PoseStamped ps;
		trajPlanner::pose p = this->trajSolver_->getPose(t);
		ps.pose.position.x = p.x;
		ps.pose.position.y = p.y;
		ps.pose.position.z = p.z;
		geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, p.yaw);
		ps.pose.orientation = quat;
		ps.header.stamp = ros::Time();
		ps.header.frame_id = "map";

		return ps;
	}

	double polyTrajOccMap::getDuration(){
		if (this->path_.size() == 1){
			return 0.0; // no duration
		}
		if (this->usePWL_){
			if (this->findValidTraj_){
				return this->trajSolver_->getTimeKnot().back();
			}
			else{
				return this->pwlTrajSolver_->getTimeKnot().back();
			}
		}

		return this->trajSolver_->getTimeKnot().back();
	}

	bool polyTrajOccMap::checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg){
		collisionSeg.clear();
		std::vector<double> timeKnot = this->trajSolver_->getTimeKnot();
		double t = 0;
		bool hasCollision = false;
		double startTime, endTime;
		for (pose p : trajectory){
			Eigen::Vector3d pEig (p.x, p.y, p.z);
			if (this->map_->isInflatedOccupied(pEig)){
				hasCollision = true;
				for (size_t i=0; i<timeKnot.size()-1; ++i){
					startTime = timeKnot[i];
					endTime = timeKnot[i+1];
					if ((t>=startTime) and (t<=endTime)){
						collisionSeg.insert(i);
						break;
					}
				}
			}
			t += delT;
		}
		return hasCollision;
	}

	void polyTrajOccMap::adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& corridorSizeVec){
		for (int collisionIdx: collisionSeg){
			corridorSizeVec[collisionIdx] = corridorSizeVec[collisionIdx] * this->fs_; 
		}
	}

	void polyTrajOccMap::trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::Path& trajectory){
		std::vector<geometry_msgs::PoseStamped> trajVec;
		for (trajPlanner::pose pTemp: trajectoryTemp){
			geometry_msgs::PoseStamped ps;
			ps.header.stamp = ros::Time();
			ps.header.frame_id = "map";
			ps.pose.position.x = pTemp.x;
			ps.pose.position.y = pTemp.y;
			ps.pose.position.z = pTemp.z;

			geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, pTemp.yaw);
			ps.pose.orientation = quat;
			trajVec.push_back(ps);
		}
		trajectory.header.stamp = ros::Time();
		trajectory.header.frame_id = "map";
		trajectory.poses = trajVec;
	}
}