/*
	File: piecewiseLinearTraj.cpp
	-----------------------------
	Function definition of piecewise linear trajetory
*/

#include <trajectory_planner/piecewiseLinearTraj.h>

namespace trajPlanner{
	pwlTraj::pwlTraj(const ros::NodeHandle& nh) : nh_(nh){
		// load parameters:
		// Desired Velocity:
		if (not this->nh_.getParam("desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[PWL Trajectory INFO]: No Desired Velocity Parameter. Use default value: 1.0m/s." << endl;
		}

		// Desired Angular Velocity:
		if (not this->nh_.getParam("desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[PWL Trajectory INFO]: No Desired Angular Velocity Parameters. Use default value: 0.5rad/s." << endl;
		}
	}

	void pwlTraj::updatePath(const nav_msgs::Path& path){
		std::vector<trajPlanner::pose> trajPath;
		for (geometry_msgs::PoseStamped p: path.poses){
			trajPlanner::pose trajP (p.pose.position.x, p.pose.position.y, p.pose.position.z);
			trajPath.push_back(trajP);
		}
		this->updatePath(trajPath);
	}

	void pwlTraj::updatePath(const std::vector<trajPlanner::pose>& path){
		this->path_ = path;
		for (int i=0; i<this->path_.size()-1; ++i){ // calculate each position's orientation (yaw). Last point does not need a orientation
			trajPlanner::pose p1 = this->path_[i];
			trajPlanner::pose p2 = this->path_[i+1];
			double yaw = std::atan2(p2.y-p1.y, p2.x-p1.x);
			this->path_[i].yaw = yaw;
		}
		this->avgTimeAllocation();
	}

	void pwlTraj::avgTimeAllocation(){
		double totalTime = 0;
		this->desiredTime_.clear();
		double currYaw = 0;
		double targetYaw = 0;
		for (int i=0; i<this->path_.size()-1; ++i){
			// rotation then move forward
			if (i != 0){
				// rotation:
				double yawDiff = trajPlanner::getYawDistance(this->path_[i+1], this->path_[i]);
				double rotationDuration = (double) yawDiff/this->desiredAngularVel_;
				totalTime += rotationDuration;
				this->desiredTime_.push_back(totalTime);

				// forward:
				double distance = trajPlanner::getPoseDistance(this->path_[i+1], this->path_[i]);
				double forwardDuration = (double) distance/this->desiredVel_;
				totalTime += forwardDuration;
				this->desiredTime_.push_back(totalTime);
			}
			else{
				double rotationDuration = 0.0;
				totalTime += rotationDuration;
				this->desiredTime_.push_back(totalTime);

				double distance = trajPlanner::getPoseDistance(this->path_[i+1], this->path_[i]);
				double forwardDuration = (double) distance/this->desiredVel_;
				totalTime += forwardDuration;
				this->desiredTime_.push_back(totalTime);
			}
		}
	}

	void pwlTraj::makePlan(){

	}

	const geometry_msgs::PoseStamped& pwlTraj::getPose(double t){
		static geometry_msgs::PoseStamped ps;
		return ps;
	}
}