/*
	FILE: polyTrajOccMap.cpp
	---------------------------
	minimum snap polynomial trajectory planner implementation based on occupancy grid map
*/

#include <trajectory_planner/polyTrajOccMap.h>

namespace trajPlanner{
	polyTrajOccMap::polyTrajOccMap(const ros::NodeHandle& nh) : nh_(nh){
		this->initParam();
		this->initPWLSolver();
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

 	}

	void polyTrajOccMap::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
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

	void polyTrajOccMap::makePlan(){
		this->findValidTraj_ = false;
		std::vector<trajPlanner::pose> trajectory;
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

		}
	}
}