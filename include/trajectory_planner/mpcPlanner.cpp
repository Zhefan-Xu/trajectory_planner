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
		this->initModules();
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

		// pointcloud resolution for clustering
		if (not this->nh_.getParam(this->ns_ + "/cloud_res", this->cloudRes_)){
			this->cloudRes_ = 0.2;
			cout << this->hint_ << ": No cloud res param. Use default: 0.2" << endl;
		}
		else{
			cout << this->hint_ << ": Cloud res is set to: " << this->cloudRes_ << endl;
		}	

		// local cloud region size x 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_x", this->regionSizeX_)){
			this->regionSizeX_ = 5.0;
			cout << this->hint_ << ": No local cloud region size x param. Use default: 5.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size x is set to: " << this->regionSizeX_ << "m" << endl;
		}	

		// local cloud region size y 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_y", this->regionSizeY_)){
			this->regionSizeY_ = 2.0;
			cout << this->hint_ << ": No local cloud region size y param. Use default: 2.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size y is set to: " << this->regionSizeY_ << "m" << endl;
		}	

		// ground height
		if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.3;
			cout << this->hint_ << ": No ground height param. Use default: 0.3m" << endl;
		}
		else{
			cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << "m" << endl;
		}		

		// safety distance
		if (not this->nh_.getParam(this->ns_ + "/safety_dist", this->safetyDist_)){
			this->safetyDist_ = 0.5;
			cout << this->hint_ << ": No safety distance param. Use default: 0.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Safety distance is set to: " << this->safetyDist_ << "m" << endl;
		}	

		// static slack variable
		if (not this->nh_.getParam(this->ns_ + "/static_constraint_slack_ratio", this->staticSlack_)){
			this->staticSlack_ = 0.5;
			cout << this->hint_ << ": No static slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Static slack variable is set to: " << this->staticSlack_ << endl;
		}				

		// dynamic slack variable
		if (not this->nh_.getParam(this->ns_ + "/dynamic_constraint_slack_ratio", this->dynamicSlack_)){
			this->dynamicSlack_ = 0.5;
			cout << this->hint_ << ": No dynamic slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Dynamic slack variable is set to: " << this->dynamicSlack_ << endl;
		}

	}

	void mpcPlanner::initModules(){
		this->obclustering_.reset(new obstacleClustering (this->cloudRes_));
	}

	void mpcPlanner::registerPub(){
		this->mpcTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/mpc_trajectory", 10);
		this->mpcTrajHistVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/traj_history", 10);
		this->localCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/local_cloud", 10);
		this->staticObstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/static_obstacles", 10);
		this->facingPub_ = this->nh_.advertise<visualization_msgs::Marker>(this->ns_ + "/clustering_facing", 10);
	}

	void mpcPlanner::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &mpcPlanner::visCB, this);
		this->clusteringTimer_ = this->nh_.createTimer(ros::Duration(0.033), &mpcPlanner::staticObstacleClusteringCB, this);
	}

	void mpcPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void mpcPlanner::staticObstacleClusteringCB(const ros::TimerEvent&){
		if (this->inputTraj_.size() == 0 or not this->stateReceived_) return;
		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		std::vector<Eigen::Vector3d> currCloud;
		double offset = 0.0;
		double angle = atan2(this->currVel_(1), this->currVel_(0));
		Eigen::Vector3d faceDirection (cos(angle), sin(angle), 0);
		Eigen::Vector3d sideDirection (-sin(angle), cos(angle), 0); // positive (left side)
		Eigen::Vector3d pOrigin = this->currPos_ - offset * faceDirection;

		// find four vextex of the bounding boxes
		Eigen::Vector3d p1, p2, p3, p4;
		p1 = pOrigin + this->regionSizeY_ * sideDirection;
		p2 = pOrigin - this->regionSizeY_ * sideDirection;
		p3 = p1 + (this->regionSizeX_ + offset) * faceDirection;
		p4 = p2 + (this->regionSizeX_ + offset) * faceDirection;

		double xStart = std::min({p1(0), p2(0), p3(0), p4(0)});
		double xEnd = std::max({p1(0), p2(0), p3(0), p4(0)});
		double yStart = std::min({p1(1), p2(1), p3(1), p4(1)});
		double yEnd = std::max({p1(1), p2(1), p3(1), p4(1)});

		for (double ix=xStart; ix<=xEnd; ix+=cloudRes_){
			for (double iy=yStart; iy<=yEnd; iy+=this->cloudRes_){
				for (double iz=this->groundHeight_; iz<=mapMax(2); iz+=this->cloudRes_){
					Eigen::Vector3d p (ix, iy, iz);
					if ((p - pOrigin).dot(faceDirection) >= 0){
						if (this->map_->isInflatedOccupied(p)){
							currCloud.push_back(p);
						}
					}
				}
			}
		}	
		this->currCloud_ = currCloud;
		this->obclustering_->run(currCloud);
		this->refinedBBoxVertices_ = this->obclustering_->getRefinedBBoxes();
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
		this->trajHist_.push_back(pos);
		this->stateReceived_ = true;
	}

	void mpcPlanner::updatePath(const nav_msgs::Path& path, double ts){
		std::vector<Eigen::Vector3d> pathTemp;
		for (int i=0; i<(path.poses.size()); ++i){
			Eigen::Vector3d p (path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
			pathTemp.push_back(p); 
		}
		this->updatePath(pathTemp, ts);
		this->trajHist_.clear();
		this->lastRefStartIdx_ = 0;
	}

	void mpcPlanner::updatePath(const std::vector<Eigen::Vector3d>& path, double ts){
		this->ts_ = ts;
		this->inputTraj_ = path;
		this->firstTime_ = true;
		this->stateReceived_ = false;
	}

	void mpcPlanner::makePlan(){
		std::ostringstream local;
		auto cout_buff = std::cout.rdbuf();
		std::cout.rdbuf(local.rdbuf());
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

		// Set up the optimal control problem
		OCP ocp (refTraj);

		DMatrix Q (8, 8);
		Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; Q(6,6) = 1000.0; Q(7,7) = 1000.0;
		ocp.minimizeLSQ(Q, h, refTraj); 
		// Contraints
		ocp.subjectTo(f); // dynamics
		double skslimit = 1.0 - pow((1 - this->staticSlack_), 2);
		double skdlimit = 1.0 - pow((1 - this->dynamicSlack_), 2);
		ocp.subjectTo( this->zRangeMin_ <= z <= this->zRangeMax_ );
		ocp.subjectTo( -this->maxVel_ <= vx <= this->maxVel_ );
		ocp.subjectTo( -this->maxVel_ <= vy <= this->maxVel_ );
		ocp.subjectTo( -this->maxVel_ <= vz <= this->maxVel_ );
		ocp.subjectTo( -this->maxAcc_ <= ax <= this->maxAcc_ );
		ocp.subjectTo( -this->maxAcc_ <= ay <= this->maxAcc_ );
		ocp.subjectTo( -this->maxAcc_ <= az <= this->maxAcc_ );
		ocp.subjectTo( 0.0 <= skd <= skdlimit);
		ocp.subjectTo( 0.0 <= sks <= skslimit);
		

		// static obstacle constraints
		std::vector<staticObstacle> staticObstacles = this->obclustering_->getStaticObstacles();
		for (int i=0; i<int(staticObstacles.size()); ++i){
			staticObstacle so = staticObstacles[i];
			double yaw = so.yaw;
			Eigen::Vector3d size = so.size/2 + Eigen::Vector3d (this->safetyDist_, this->safetyDist_, this->safetyDist_);
			Eigen::Vector3d centroid = so.centroid;

			if (size(0) == 0 or size(1) == 0 or size(2) == 0) continue;
			ocp.subjectTo(pow((x - centroid(0))*cos(yaw) + (y - centroid(1))*sin(yaw), 2)/pow(size(0), 2) + pow(-(x - centroid(0))*sin(yaw) + (y - centroid(1))*cos(yaw), 2)/pow(size(1), 2) + pow(z - centroid(2), 2)/pow(size(2), 2) - 1  + sks >= 0 );
		}

		// Algorithm
		RealTimeAlgorithm RTalgorithm(ocp, this->ts_);
		RTalgorithm.set(PRINTLEVEL, 0);
		if (not this->firstTime_){
			RTalgorithm.initializeDifferentialStates(this->currentStatesSol_);
		}

		RTalgorithm.set(MAX_NUM_ITERATIONS, 1);
		// RTalgorithm.set(KKT_TOLERANCE, 3e-4);
		RTalgorithm.set(TERMINATE_AT_CONVERGENCE, BT_TRUE);

		// Solve
		DVector currentState ({this->currPos_(0), this->currPos_(1), this->currPos_(2), this->currVel_(0), this->currVel_(1), this->currVel_(2)});
		RTalgorithm.solve(0.0, currentState); // start time and t0
		RTalgorithm.getDifferentialStates(this->currentStatesSol_);
		RTalgorithm.getControls(this->currentControlsSol_);
		this->firstTime_ = false;
		clearAllStaticCounters();
		// cout << this->currentControlsSol_ << endl;
		std::cout.rdbuf(cout_buff);
	}


	void mpcPlanner::getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj){
		// find the nearest position in the reference trajectory
		double leastDist = std::numeric_limits<double>::max();
		double maxForwardTime = 3.0; // # 3.0s ahead
		int maxForwardIdx = maxForwardTime/this->ts_;
		int startIdx = this->lastRefStartIdx_;
		for (int i=this->lastRefStartIdx_; i<this->lastRefStartIdx_+maxForwardIdx; ++i){
			double dist = (this->currPos_ - this->inputTraj_[i]).norm();
			if (dist < leastDist){
				leastDist = dist;
				startIdx = i;
			}
		}
		this->lastRefStartIdx_ = startIdx; // update start idx

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
		this->publishHistoricTrajectory();
		this->publishLocalCloud();
		this->publishStaticObstacles();
	}

	void mpcPlanner::publishMPCTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path traj;
			this->getTrajectory(traj);
			this->mpcTrajVisPub_.publish(traj);
		}
	}

	void mpcPlanner::publishHistoricTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path histTraj;
			histTraj.header.frame_id = "map";
			for (int i=0; i<int(this->trajHist_.size()); ++i){
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = this->trajHist_[i](0);
				ps.pose.position.y = this->trajHist_[i](1);
				ps.pose.position.z = this->trajHist_[i](2);
				histTraj.poses.push_back(ps);
			}
			this->mpcTrajHistVisPub_.publish(histTraj);
		}
	}

	void mpcPlanner::publishLocalCloud(){
		if (this->currCloud_.size() != 0){
			pcl::PointXYZ pt;
			pcl::PointCloud<pcl::PointXYZ> cloud;

			for (int i=0; i<int(this->currCloud_.size()); ++i){
				pt.x = this->currCloud_[i](0);
				pt.y = this->currCloud_[i](1);
				pt.z = this->currCloud_[i](2);
				cloud.push_back(pt);
			}

			cloud.width = cloud.points.size();
			cloud.height = 1;
			cloud.is_dense = true;
			cloud.header.frame_id = "map";

			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			this->localCloudPub_.publish(cloudMsg);		
		}
	}

	void mpcPlanner::publishStaticObstacles(){
		if (this->refinedBBoxVertices_.size() != 0){
		    visualization_msgs::Marker line;
		    visualization_msgs::MarkerArray lines;

		    line.header.frame_id = "map";
		    line.type = visualization_msgs::Marker::LINE_LIST;
		    line.action = visualization_msgs::Marker::ADD;
		    line.ns = "mpc_static_obstacles";  
		    line.scale.x = 0.06;
		    line.color.r = 0;
		    line.color.g = 1;
		    line.color.b = 1;
		    line.color.a = 1.0;
		    line.lifetime = ros::Duration(0.1);
		    Eigen::Vector3d vertex_pose;
		    for(int i=0; i<int(this->refinedBBoxVertices_.size()); ++i){
		        bboxVertex v = this->refinedBBoxVertices_[i];
		        std::vector<geometry_msgs::Point> verts;
		        geometry_msgs::Point p;

				for (int j=0; j<int(v.vert.size());++j){
					p.x = v.vert[j](0); p.y = v.vert[j](1); p.z = v.vert[j](2);
		        	verts.push_back(p);
				}

		        int vert_idx[12][2] = {
		            {0,1},
		            {1,2},
		            {2,3},
		            {0,3},
		            {0,4},
		            {1,5},
		            {3,7},
		            {2,6},
		            {4,5},
		            {5,6},
		            {4,7},
		            {6,7}
		        };
		        for (int j=0;j<12;++j){
		            line.points.push_back(verts[vert_idx[j][0]]);
		            line.points.push_back(verts[vert_idx[j][1]]);
		        }
		        lines.markers.push_back(line);
		        line.id++;
		    }
		    this->staticObstacleVisPub_.publish(lines);		


		    // facing direction	
		    visualization_msgs::Marker currPoseMarker;
	    	currPoseMarker.header.frame_id = "map";
			currPoseMarker.header.stamp = ros::Time();
			currPoseMarker.ns = "currPoseVis";
			currPoseMarker.id = 0;
			currPoseMarker.type = visualization_msgs::Marker::ARROW;
			currPoseMarker.action = visualization_msgs::Marker::ADD;
			currPoseMarker.pose.position.x = this->currPos_(0);
			currPoseMarker.pose.position.y = this->currPos_(1);
			currPoseMarker.pose.position.z = this->currPos_(2);
			double angle = atan2(this->currVel_(1), this->currVel_(0));
			currPoseMarker.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, angle);
			currPoseMarker.lifetime = ros::Duration(0.1);
			currPoseMarker.scale.x = 0.4;
			currPoseMarker.scale.y = 0.2;
			currPoseMarker.scale.z = 0.2;
			currPoseMarker.color.a = 1.0;
			currPoseMarker.color.r = 0.0;
			currPoseMarker.color.g = 0.5;
			currPoseMarker.color.b = 1.0;
			this->facingPub_.publish(currPoseMarker);
		}
	}
}