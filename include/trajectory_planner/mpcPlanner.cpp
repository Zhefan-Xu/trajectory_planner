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
		this->registerPub();
		this->registerCallback();
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

		// constraint slack variable
		if (not this->nh_.getParam(this->ns_ + "/constraint_slack_ratio", this->constraintSlackRatio_)){
			this->constraintSlackRatio_ = 0.4;
			cout << this->hint_ << ": No constraint slack ratio param. Use default: 0.4" << endl;
		}
		else{
			cout << this->hint_ << ": Constraint slack ratio is set to: " << this->constraintSlackRatio_ << endl;
		}	

		// mininimum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_min", this->zRangeMin_)){
			this->zRangeMin_ = 0.7;
			cout << this->hint_ << ": No z range min param. Use default: 0.7m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range min is set to: " << this->zRangeMin_ << "m" << endl;
		}

		// maximum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_max", this->zRangeMax_)){
			this->zRangeMax_ = 1.2;
			cout << this->hint_ << ": No z range max param. Use default: 1.2m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range max is set to: " << this->zRangeMax_ << "m" << endl;
		}					
	}

	void mpcPlanner::registerPub(){
		this->mpcTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("mpc/mpc_trajectory", 10);
	}

	void mpcPlanner::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &mpcPlanner::visCB, this);
	}

	void mpcPlanner::updateMaxVel(double maxVel){
		this->maxVel_ = maxVel;
	}

	void mpcPlanner::updateMaxAcc(double maxAcc){
		this->maxAcc_ = maxAcc;
	}

	void mpcPlanner::updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel){
		this->currPos_ = pos;
		this->currVel_ = vel;
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
		this->firstTime_ = true;
	}

	void mpcPlanner::makePlan(){
		cout << "1" << endl;
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
		Control skd;
		Control sks;
		
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
		h << skd;
		h << sks;

		// Reference Trajectory
		VariablesGrid refTraj = this->getReferenceTraj();
		cout << refTraj << endl;

		// Set up the optimal control problem
		OCP ocp (refTraj);
		DMatrix Q (8, 8);
		Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; Q(6,6) = 1000.0; Q(7,7) = 1000.0;
		ocp.minimizeLSQ(Q, h, refTraj); 

		// Contraints
		ocp.subjectTo(f); // dynamics
		double sklimit = 1.0 - pow((1 - this->constraintSlackRatio_), 2);
		ocp.subjectTo( this->zRangeMin_ <= z <= this->zRangeMax_ );
		ocp.subjectTo( -this->maxVel_ <= vx <= this->maxVel_ );
		ocp.subjectTo( -this->maxVel_ <= vy <= this->maxVel_ );
		ocp.subjectTo( -this->maxVel_ <= vz <= this->maxVel_ );
		ocp.subjectTo( -this->maxAcc_ <= ax <= this->maxAcc_ );
		ocp.subjectTo( -this->maxAcc_ <= ay <= this->maxAcc_ );
		ocp.subjectTo( -this->maxAcc_ <= az <= this->maxAcc_ );
		ocp.subjectTo( 0.0 <= skd <= sklimit);
		ocp.subjectTo( 0.0 <= sks <= sklimit);

		// Algorithm
		RealTimeAlgorithm RTalgorithm(ocp, this->ts_);
		if (not this->firstTime_){
			RTalgorithm.initializeDifferentialStates(this->currentStatesSol_);
		}

		RTalgorithm.set(MAX_NUM_ITERATIONS, 1);
		RTalgorithm.set(KKT_TOLERANCE, 3e-4);
		RTalgorithm.set(TERMINATE_AT_CONVERGENCE, BT_TRUE);

		// Solve
		cout << "here1" << endl;
		DVector currentState ({this->currPos_(0), this->currPos_(1), this->currPos_(2), this->currVel_(0), this->currVel_(1), this->currVel_(2)});
		cout << "here2" << endl;
		RTalgorithm.solve(0.0, currentState); // start time and t0
		cout << "here3" << endl;
		RTalgorithm.getDifferentialStates(this->currentStatesSol_);
		RTalgorithm.getControls(this->currentControlsSol_);
		this->firstTime_ = false;
		clearAllStaticCounters();
		cout << "2" << endl;
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
			if (i < int(this->inputTraj_.size())){
				referenceTraj.push_back(this->inputTraj_[i]);
			}
			else{
				referenceTraj.push_back(this->inputTraj_.back());
			}
		}
	}

	VariablesGrid mpcPlanner::getReferenceTraj(){
		std::vector<Eigen::Vector3d> referenceTraj;
		this->getReferenceTraj(referenceTraj);

		VariablesGrid r (8, 0);
		double t = 0.0;
		for (int i=0; i<int(referenceTraj.size()); ++i){
			DVector p (8);
			// pos
			p(0) = referenceTraj[i](0);
			p(1) = referenceTraj[i](1);
			p(2) = referenceTraj[i](2);
			// control input
			p(3) = 0.0;
			p(4) = 0.0;
			p(5) = 0.0;
			// slack variables
			p(6) = 0.0;
			p(7) = 0.0;
			r.addVector(p, t);
			t += this->ts_;
		}
		return r;
	}

	void mpcPlanner::getTrajectory(std::vector<Eigen::Vector3d>& traj){
		traj.clear();
		for (int i=0; i<this->horizon_; ++i){
			DVector states = this->currentStatesSol_.getVector(i);
			Eigen::Vector3d p (states(0), states(1), states(2));
			traj.push_back(p);
		}
	}

	void mpcPlanner::getTrajectory(nav_msgs::Path& traj){
		traj.header.frame_id = "map";
		std::vector<Eigen::Vector3d> trajTemp;
		this->getTrajectory(trajTemp);
		for (int i=0; i<int(trajTemp.size()); ++i){
			geometry_msgs::PoseStamped ps;
			ps.pose.position.x = trajTemp[i](0);
			ps.pose.position.y = trajTemp[i](1);
			ps.pose.position.z = trajTemp[i](2);
			traj.poses.push_back(ps);
		}
	}

	Eigen::Vector3d mpcPlanner::getPos(double t){
		int idx = int(t/this->ts_);
		idx = std::max(0, std::min(idx, this->horizon_-1));
		DVector states = this->currentStatesSol_.getVector(idx);
		Eigen::Vector3d p (states(0), states(1), states(2));
		return p;
	}

	Eigen::Vector3d mpcPlanner::getVel(double t){
		int idx = int(t/this->ts_);
		idx = std::max(0, std::min(idx, this->horizon_-1));
		DVector states = this->currentStatesSol_.getVector(idx);
		Eigen::Vector3d v (states(3), states(4), states(5));
		return v;
	}

	Eigen::Vector3d mpcPlanner::getAcc(double t){
		int idx = int(t/this->ts_);
		idx = std::max(0, std::min(idx, this->horizon_-1));
		DVector states = this->currentControlsSol_.getVector(idx);
		Eigen::Vector3d a (states(0), states(1), states(2));
		return a;
	}

	void mpcPlanner::visCB(const ros::TimerEvent&){
		this->publishMPCTrajectory();
	}

	void mpcPlanner::publishMPCTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path traj;
			this->getTrajectory(traj);
			this->mpcTrajVisPub_.publish(traj);
		}
	}
}