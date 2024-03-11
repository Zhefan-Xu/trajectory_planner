/*
	FILE: mpcPlanner.cpp
	-----------------------------
	mpc trajectory solver implementation based on occupancy grid map 
*/

#include <trajectory_planner/mpcPlanner.h>

namespace trajPlanner{
	mpcPlanner::mpcPlanner(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "mpc_planner";
		this->hint_ = "[MPCPlanner]";
		this->initParam();
	}

	void mpcPlanner::initParam(){ 
		// planning horizon
		if (not this->nh_.getParam(this->ns_ + "/horizon", this->horizon_)){
			this->horizon_ = 20;
			cout << this->hint_ << ": No planning horizon param. Use default: 20" << endl;
		}
		else{
			cout << this->hint_ << ": Planning horizon is set to: " << this->horizon_ << endl;
		}			
	}

	void mpcPlanner::updateCurrPosition(const Eigen::Vector3d& pos){
		this->currPos_ = pos;
	}

	void mpcPlanner::updatePath(const nav_msgs::Path& path, double ts){
		std::vector<Eigen::Vector3d> pathTemp;
		for (int i=0; i<(path.poses.size()); ++i){
			Eigen::Vector3d p (path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
			pathTemp.push_back(p); 
		}
		this->updatePath(pathTemp, ts);
	}

	void mpcPlanner::updatePath(const std::vector<Eigen::Vector3d>& path, double ts){
		this->ts_ = ts;
		this->inputTraj_ = path;
	}

	void mpcPlanner::makePlan(){
		// States
		DifferentialState x;
		DifferentialState y;
		DifferentialState z;
		DifferentialState vx;
		DifferentialState vy;
		DifferentialState vz;	

		// Control Input
		Control ax;
		Control ay;
		Control az;
		
		// Dynamics Model
		DifferentialEquation f;
		f << dot(x) == vx;
		f << dot(y) == vy;
		f << dot(z) == vz;
		f << dot(vx) == ax; 
		f << dot(vy) == ay;
		f << dot(vz) == az;
		
		// Objective Function
		Function h;
		h << x;
		h << y;
		h << z;
		h << ax;
		h << ay;
		h << az;

	}


	void mpcPlanner::getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj){
		// find the nearest position in the reference trajectory
		double leastDist = std::numeric_limits<double>::max();
		int startIdx = 0;
		for (int i=0; i<int(this->inputTraj_.size()); ++i){
			double dist = (this->currPos_ - this->inputTraj_[i]).norm();
			if (dist < leastDist){
				leastDist = dist;
				startIdx = i;
			}
		}

		referenceTraj.clear();
		for (int i=startIdx; i<startIdx+this->horizon_; ++i){
			if (i < int(referenceTraj.size())){
				referenceTraj.push_back(this->inputTraj_[i]);
			}
			else{
				referenceTraj.push_back(this->inputTraj_.back());
			}
		}

	}
}