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

	void pwlTraj::updatePath(const nav_msgs::Path& path, bool useYaw){
		std::vector<trajPlanner::pose> trajPath;
		for (geometry_msgs::PoseStamped p: path.poses){
			trajPlanner::pose trajP;
			if (not useYaw){
				trajPlanner::pose trajPTemp (p.pose.position.x, p.pose.position.y, p.pose.position.z);
				trajP = trajPTemp;
			}
			else{
				trajPlanner::pose trajPTemp (p.pose.position.x, p.pose.position.y, p.pose.position.z, trajPlanner::rpy_from_quaternion(p.pose.orientation));
				trajP = trajPTemp;
			}

			trajPath.push_back(trajP);
		}
		this->updatePath(trajPath, useYaw);
	}

	void pwlTraj::updatePath(const std::vector<trajPlanner::pose>& path, bool useYaw){
		this->path_ = path;
		double yaw;
		if (not useYaw){
			for (int i=0; i<this->path_.size()-1; ++i){ // calculate each position's orientation (yaw). Last point does not need a orientation
				trajPlanner::pose p1 = this->path_[i];
				trajPlanner::pose p2 = this->path_[i+1];
				yaw = std::atan2(p2.y-p1.y, p2.x-p1.x);

				this->path_[i].yaw = yaw;
			}
			this->path_.back().yaw = yaw;
		}

		this->avgTimeAllocation(useYaw);
	}

	void pwlTraj::adjustHeading(const geometry_msgs::Quaternion& quat){
		double yaw = trajPlanner::rpy_from_quaternion(quat);
		this->adjustHeading(yaw);
	}

	void pwlTraj::adjustHeading(double yaw){// sometimes the heading is not correct
		trajPlanner::pose p1 = this->path_[0];
		trajPlanner::pose p (p1.x, p1.y, p1.z, yaw);
		this->path_.insert(this->path_.begin(), 0, p);
		this->path_.insert(this->path_.begin(), 0, p);
		this->path_.insert(this->path_.begin(), 0, p); // we need three
		this->avgTimeAllocation();
	}

	void pwlTraj::avgTimeAllocation(bool useYaw){
		double totalTime = 0;
		this->desiredTime_.clear();
		double currYaw = 0;
		double targetYaw = 0;
		for (int i=0; i<this->path_.size()-1; ++i){
			// rotation then move forward
			if (i != 0){
				// rotation:
				double yawDiff = trajPlanner::getYawDistance(this->path_[i-1], this->path_[i]);
				double rotationDuration = (double) yawDiff/this->desiredAngularVel_;
				totalTime += rotationDuration;
				this->desiredTime_.push_back(totalTime);

				// forward:
				double distance = trajPlanner::getPoseDistance(this->path_[i], this->path_[i+1]);
				double forwardDuration = (double) distance/this->desiredVel_;
				totalTime += forwardDuration;
				this->desiredTime_.push_back(totalTime);
			}
			else{
				// at t = 0, we don't need to do rotation
				double rotationDuration = 0.0;
				totalTime += rotationDuration;
				this->desiredTime_.push_back(totalTime);

				double distance = trajPlanner::getPoseDistance(this->path_[0], this->path_[1]);
				double forwardDuration = (double) distance/this->desiredVel_;
				totalTime += forwardDuration;
				this->desiredTime_.push_back(totalTime);
			}
		}

		if (useYaw){
			int lastIdx = this->path_.size() - 1;
			double yawDiff = trajPlanner::getYawDistance(this->path_[lastIdx-1], this->path_[lastIdx]);
			double rotationDuration = (double) yawDiff/this->desiredAngularVel_;
			totalTime += rotationDuration;
			this->desiredTime_.push_back(totalTime);
		}
	}

	void pwlTraj::makePlan(nav_msgs::Path& trajectory, double delT){
		std::vector<geometry_msgs::PoseStamped> trajVec;
		for (double t=0; t<=this->desiredTime_.back(); t+=delT){
			geometry_msgs::PoseStamped ps = this->getPose(t);
			trajVec.push_back(ps);
		}
		trajectory.poses = trajVec;
		trajectory.header.frame_id = "map";
		trajectory.header.stamp = ros::Time();
	}

	void pwlTraj::makePlan(std::vector<trajPlanner::pose>& trajectory, double delT){
		trajectory.clear();
		for (double t=0; t<=this->desiredTime_.back(); t+=delT){
			trajPlanner::pose p;
			geometry_msgs::PoseStamped ps = this->getPose(t);
			double yaw = trajPlanner::rpy_from_quaternion(ps.pose.orientation);
			p.x = ps.pose.position.x;
			p.y = ps.pose.position.y;
			p.z = ps.pose.position.z;
			p.yaw = yaw;
			trajectory.push_back(p);
		}
	}

	geometry_msgs::PoseStamped pwlTraj::getPose(double t){
		geometry_msgs::PoseStamped ps;
		if (t >= this->getDuration()){
			trajPlanner::pose lastP = this->path_[this->path_.size()-1];
			ps.pose.position.x = lastP.x;
			ps.pose.position.y = lastP.y;
			ps.pose.position.z = lastP.z;
			ps.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, lastP.yaw);
			ps.header.frame_id = "map";
			ps.header.stamp = ros::Time::now();
			return ps;
		}


		for (int i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				if (i % 2 == 1){ // it is in a rotation period
					// find the corresponding point
					int pointIdx = (i-1) / 2;
					trajPlanner::pose pCurr = this->path_[pointIdx];
					trajPlanner::pose pTarget = this->path_[pointIdx+1];

					// Interpolation between yaw:
					double yawDiff = pTarget.yaw - pCurr.yaw; // difference between yaw
					double direction;
					double yawDiffAbs = std::abs(yawDiff);
					if ((yawDiffAbs <= PI_const) and (yawDiff>0)){
						direction = 1.0; // counter clockwise
					} 
					else if ((yawDiffAbs <= PI_const) and (yawDiff<0)){
						direction = -1.0; // clockwise
					}
					else if ((yawDiffAbs > PI_const) and (yawDiff>0)){
						direction = -1.0; // rotate in clockwise direction
						yawDiffAbs = 2 * PI_const - yawDiffAbs;
					}
					else if ((yawDiffAbs > PI_const) and (yawDiff<0)){
						direction = 1.0; // counter clockwise
						yawDiffAbs = 2 * PI_const - yawDiffAbs;
					}

					ps.pose.position.x = pTarget.x;
					ps.pose.position.y = pTarget.y;
					ps.pose.position.z = pTarget.z;

					double currYaw = pCurr.yaw + direction * (t-startTime)/(endTime-startTime) * yawDiffAbs;
					geometry_msgs::Quaternion quat = trajPlanner::quaternion_from_rpy(0, 0, currYaw);
					ps.pose.orientation = quat; 
				}
				else{ // it is in forward motion period
					int pointIdx = i / 2;
					trajPlanner::pose pCurr = this->path_[pointIdx];
					trajPlanner::pose pTarget = this->path_[pointIdx+1];

					ps.pose.position.x = pCurr.x +  (t-startTime) * (pTarget.x - pCurr.x)/(endTime-startTime);
					ps.pose.position.y = pCurr.y +  (t-startTime) * (pTarget.y - pCurr.y)/(endTime-startTime);
					ps.pose.position.z = pCurr.z +  (t-startTime) * (pTarget.z - pCurr.z)/(endTime-startTime);

					double currYaw = pCurr.yaw;
					geometry_msgs::Quaternion quat = trajPlanner::quaternion_from_rpy(0, 0, currYaw);
					ps.pose.orientation = quat; 
				}
				break;
			}
		}
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		return ps;
	}

	std::vector<double> pwlTraj::getTimeKnot(){
		return this->desiredTime_;
	}

	double pwlTraj::getDuration(){
		if (this->desiredTime_.size() != 0){
			return this->desiredTime_[this->desiredTime_.size()-1];
		}
		else{
			return -1.0;
		}
	}
}