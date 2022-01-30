/*
	file: polyTrajOctomap.cpp
	-------------------------
	function definition of minimum snap polynomial trajectory planner
*/
#include <trajectory_planner/polyTrajOctomap.h>

namespace trajPlanner{
	polyTrajOctomap::polyTrajOctomap(){}

	polyTrajOctomap::polyTrajOctomap(const ros::NodeHandle& nh, std::vector<double> collisionBox, int degree, double veld, int diffDegree, double regularizationWeights, double mapRes, int maxIter, double initR, double fs)
	: nh_(nh), collisionBox_(collisionBox), degree_(degree), veld_(veld), diffDegree_(diffDegree), regularizationWeights_(regularizationWeights), mapRes_(mapRes), maxIter_(maxIter), initR_(initR), fs_(fs){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// visualize:
		this->trajVisPub_= this->nh_.advertise<nav_msgs::Path>("/trajectory", 1);
		this->samplePointVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("trajectory_sp", 1);
		this->trajVisWorker_ = std::thread(&polyTrajOctomap::publishTrajectory, this);
		this->samplePointVisWorker_ = std::thread(&polyTrajOctomap::publishSamplePoint, this);
	}

	void polyTrajOctomap::updateMap(){
		octomap_msgs::GetOctomap mapSrv;
		bool service_success = this->mapClient_.call(mapSrv);
		ros::Rate rate(10);
		while (not service_success and ros::ok()){
			service_success = this->mapClient_.call(mapSrv);
			ROS_INFO("[Trajectory Planner INFO]: Wait for Octomap Service...");
			rate.sleep();
		}
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		cout << "[Trajectory Planner INFO]: Map updated!" << endl;
	}

	polyTrajSolver*  polyTrajOctomap::initSolver(){
		return new polyTrajSolver (this->degree_, this->diffDegree_, this->veld_);
	}

	void polyTrajOctomap::freeSolver(){
		delete this->trajSolver_;
	}

	void polyTrajOctomap::updatePath(const std::vector<pose>& path){
		this->path_ = path;
	}

	void polyTrajOctomap::makePlan(std::vector<pose>& trajectory, double delT){
		if (this->path_.size() == 1){
			trajectory = this->path_;
			return;
		}
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->solve();
		this->trajSolver_->getTrajectory(trajectory, delT);
		this->updateTrajVisMsg(trajectory);
		this->freeSolver();
	}


	bool polyTrajOctomap::checkCollision(const octomap::point3d& p){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		for (double x=xmin; x<xmax; x+=this->mapRes_){
			for (double y=ymin; y<ymax; y+=this->mapRes_){
				for (double z=zmin; z<zmax; z+=this->mapRes_){
					if (!this->checkCollisionPoint(octomap::point3d (x, y, z))){
						// do nothing
					}
					else{
						return true;
					}
				}
			}
		}
		return false;
	}


	bool polyTrajOctomap::checkCollisionPoint(const octomap::point3d &p){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			return true;
		}
		return this->map_->isNodeOccupied(nptr);
	}

	bool polyTrajOctomap::checkCollisionPoint(const pose& pTraj){
		octomap::point3d p;
		this->pose2Octomap(pTraj, p);
		return this->checkCollisionPoint(p);
	}

	bool polyTrajOctomap::checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2){
		std::vector<octomap::point3d> ray;
		this->map_->computeRay(p1, p2, ray);
		for (octomap::point3d p: ray){
			if (this->checkCollision(p)){
				return true;
			}
		}
		return false;
	}

	bool polyTrajOctomap::checkCollisionLine(const pose& pTraj1, const pose& pTraj2){
		octomap::point3d p1, p2;
		this->pose2Octomap(pTraj1, p1);
		this->pose2Octomap(pTraj2, p2);
		return this->checkCollisionLine(p1, p2);
	}

	bool polyTrajOctomap::checkCollisionTraj(const std::vector<pose>& trajectory, std::vector<int>& collisionIdx){
		octomap::point3d p;
		int count = 0;
		bool hasColllision;
		for (pose pTraj: trajectory){
			this->pose2Octomap(pTraj, p);
			if (this->checkCollisionPoint(p)){
				hasColllision = true;
				collisionIdx.push_back(count);
			}
			++count;
		}
		return hasColllision;
	}

	double polyTrajOctomap::getDegree(){
		return this->degree_;
	}

	double polyTrajOctomap::getVelD(){
		return this->veld_;
	}

	double polyTrajOctomap::getDiffDegree(){
		return this->diffDegree_;
	}

	double polyTrajOctomap::getInitialRadius(){
		return this->initR_;
	}

	double polyTrajOctomap::getShrinkFactor(){
		return this->fs_;
	}

	void polyTrajOctomap::updateTrajVisMsg(const std::vector<pose>& trajectory){
		std::vector<geometry_msgs::PoseStamped> trajVisVec;
		for (pose p: trajectory){
			geometry_msgs::PoseStamped ps;
			ps.header.frame_id = "map";
			ps.header.stamp = ros::Time::now();
			ps.pose.position.x = p.x;
			ps.pose.position.y = p.y;
			ps.pose.position.z = p.z; 
			trajVisVec.push_back(ps);
		}
		this->trajVisMsg_.poses = trajVisVec;

		std::vector<visualization_msgs::Marker> samplePointVec;
		visualization_msgs::Marker samplePoint;
		int count = 0;
		for (pose p: trajectory){
			samplePoint.header.frame_id = "map";
			samplePoint.header.stamp = ros::Time();
			samplePoint.ns = "sample_point";
			samplePoint.id = count;
			samplePoint.type = visualization_msgs::Marker::ARROW;
			samplePoint.action = visualization_msgs::Marker::ADD;
			samplePoint.pose.position.x = p.x;
			samplePoint.pose.position.y = p.y;
			samplePoint.pose.position.z = p.z;
			samplePoint.pose.orientation = quaternion_from_rpy(0, 0, p.yaw);
			samplePoint.lifetime = ros::Duration(0.5);
			samplePoint.scale.x = 0.1;
			samplePoint.scale.y = 0.2;
			samplePoint.scale.z = 0.2;
			samplePoint.color.a = 0.5;
			samplePoint.color.r = 0.7;
			samplePoint.color.g = 1.0;
			samplePoint.color.b = 0.0;
			samplePointVec.push_back(samplePoint);
			++count;
		}
		this->samplePointMsg_.markers = samplePointVec;
	}

	void polyTrajOctomap::publishTrajectory(){
		ros::Rate r (10);
		while (ros::ok()){
			this->trajVisMsg_.header.frame_id = "map";
			this->trajVisMsg_.header.stamp = ros::Time::now();
			this->trajVisPub_.publish(this->trajVisMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::publishSamplePoint(){
		ros::Rate r (10);
		while (ros::ok()){
			this->samplePointVisPub_.publish(this->samplePointMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::pose2Octomap(const pose& pTraj, octomap::point3d& p){
		p.x() = pTraj.x;
		p.y() = pTraj.y;
		p.z() = pTraj.z;
	}
}
