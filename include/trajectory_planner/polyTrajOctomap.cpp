/*
	file: polyTrajOctomap.cpp
	-------------------------
	function definition of minimum snap polynomial trajectory planner
*/
#include <trajectory_planner/polyTrajOctomap.h>

namespace trajPlanner{
	polyTrajOctomap::polyTrajOctomap(){}

	polyTrajOctomap::polyTrajOctomap(const ros::NodeHandle& nh, std::vector<double> collisionBox, int degree, int diffDegree, int continuityDegree, double veld, double regularizationWeights, double mapRes, int maxIter, double initR, double fs)
	: nh_(nh), collisionBox_(collisionBox), degree_(degree), diffDegree_(diffDegree),  continuityDegree_(continuityDegree), veld_(veld), regularizationWeights_(regularizationWeights), mapRes_(mapRes), maxIter_(maxIter), initR_(initR), fs_(fs){
		// load parameters:
		


		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// visualize:
		this->trajVisPub_= this->nh_.advertise<nav_msgs::Path>("/trajectory", 1);
		this->samplePointVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_sp", 1);
		this->waypointVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/waypoint", 1);
		this->corridorVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/corridor", 1);
		this->trajVisWorker_ = std::thread(&polyTrajOctomap::publishTrajectory, this);
		this->samplePointVisWorker_ = std::thread(&polyTrajOctomap::publishSamplePoint, this);
		this->waypointVisWorker_ = std::thread(&polyTrajOctomap::publishWaypoint, this);
		this->corridorVisWorker_ = std::thread(&polyTrajOctomap::publishCorridor, this);
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
		return new polyTrajSolver (this->degree_, this->diffDegree_, this->continuityDegree_,this->veld_);
	}

	void polyTrajOctomap::freeSolver(){
		delete this->trajSolver_;
	}

	void polyTrajOctomap::updatePath(const std::vector<pose>& path){
		this->path_ = path;
	}

	void polyTrajOctomap::insertWaypoint(const std::set<int>& seg){
		for (std::set<int>::reverse_iterator rit = seg.rbegin(); rit != seg.rend(); rit++){ // reverse order can solve the segment number change problem
			int idx = *rit;
			trajPlanner::pose p1 = this->path_[idx];
			trajPlanner::pose p2 = this->path_[idx+1];
			trajPlanner::pose pMid = trajPlanner::pose ((p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2);
			this->path_.insert(this->path_.begin()+idx+1, pMid);
		}
	}

	void polyTrajOctomap::adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& corridorSizeVec){
		for (int collisionIdx: collisionSeg){
			corridorSizeVec[collisionIdx] = corridorSizeVec[collisionIdx] * this->fs_; 
		}
	}

	void polyTrajOctomap::makePlan(std::vector<pose>& trajectory, double delT){
		ros::Time startTime = ros::Time::now();
		
		// this->makePlanAddingWaypoint(trajectory, delT);
		this->makePlanCorridorConstraint(trajectory, delT);

		ros::Time endTime = ros::Time::now();
		double dT = (endTime - startTime).toSec();
		cout << "[Trajectory Planner INFO]: Time: " << dT << "s."<< endl;
	}

	void polyTrajOctomap::makePlanAddingWaypoint(std::vector<pose>& trajectory, double delT){	
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			return;
		}
		this->trajSolver_ = this->initSolver();
		int countIter = 0;
		bool valid = false;
		while (ros::ok() and not valid){
			this->trajSolver_->updatePath(this->path_);
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, delT);
			std::set<int> collisionSeg;
			valid = not this->checkCollisionTraj(trajectory, delT, collisionSeg);
			if (not valid){
				this->insertWaypoint(collisionSeg);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}
		this->updateTrajVisMsg(trajectory);
		this->freeSolver();

		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best." << endl;	
		}
	}

	void polyTrajOctomap::makePlanCorridorConstraint(std::vector<pose>& trajectory, double delT){
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			return;
		}
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updatePath(this->path_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec;
		for (int i=0; i<this->path_.size()-1; ++i){
			corridorSizeVec.push_back(corridorSize);
		} 

		double corridorRes = 5.0; // 1 corridor box per 1 second
		int countIter = 0;
		bool valid = false;
		while (ros::ok() and not valid){
			this->trajSolver_->setCorridorConstraint(corridorSizeVec, corridorRes);
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, delT);
			std::set<int> collisionSeg;
			valid  = not this->checkCollisionTraj(trajectory, delT, collisionSeg);
			if (not valid){
				this->adjustCorridorSize(collisionSeg, corridorSizeVec);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}

		this->updateTrajVisMsg(trajectory);
		std::vector<double> corridorSizeVecVis;
		std::vector<std::unordered_map<double, trajPlanner::pose>> segToTimePoseVis;
		this->trajSolver_->getCorridor(segToTimePoseVis, corridorSizeVecVis);
		this->updateCorridorVisMsg(segToTimePoseVis, corridorSizeVecVis);
		this->freeSolver();

		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best." << endl;	
		}
	}

	bool polyTrajOctomap::checkCollision(const octomap::point3d& p){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		for (double x=xmin; x<=xmax; x+=this->mapRes_){
			for (double y=ymin; y<=ymax; y+=this->mapRes_){
				for (double z=zmin; z<=zmax; z+=this->mapRes_){
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


	bool polyTrajOctomap::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			if (not ignoreUnknown){
				return true;
			}
			else{
				return false;
			}
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
		if (this->checkCollision(p2)){
			return true;
		}

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
		bool hasColllision = false;
		for (pose pTraj: trajectory){
			this->pose2Octomap(pTraj, p);
			if (this->checkCollision(p)){
				hasColllision = true;
				collisionIdx.push_back(count);
			}
			++count;
		}
		return hasColllision;
	}

	bool polyTrajOctomap::checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg){
		collisionSeg.clear();
		std::vector<double> timeKnot = this->trajSolver_->getTimeKnot();
		octomap::point3d p;
		double t = 0;
		bool hasColllision = false;
		for (pose pTraj: trajectory){
			this->pose2Octomap(pTraj, p);
			if (this->checkCollision(p)){
				hasColllision = true;
				for (int i=0; i<timeKnot.size()-1; ++i){
					double startTime = timeKnot[i];
					double endTime = timeKnot[i+1];
					if ((t>=startTime) and (t<=endTime)){
						collisionSeg.insert(i);
						break;
					}
				}
			}
			t += delT;
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

		std::vector<visualization_msgs::Marker> waypointVec;
		visualization_msgs::Marker waypoint;
		int waypointCount = 0;
		for (pose p: this->path_){
			waypoint.header.frame_id = "map";
			waypoint.header.stamp = ros::Time();
			waypoint.ns = "waypoint";
			waypoint.id = waypointCount;
			waypoint.type = visualization_msgs::Marker::SPHERE;
			waypoint.action = visualization_msgs::Marker::ADD;
			waypoint.pose.position.x = p.x;
			waypoint.pose.position.y = p.y;
			waypoint.pose.position.z = p.z;
			waypoint.lifetime = ros::Duration(0.5);
			waypoint.scale.x = 0.3;
			waypoint.scale.y = 0.3;
			waypoint.scale.z = 0.3;
			waypoint.color.a = 0.5;
			waypoint.color.r = 0.7;
			waypoint.color.g = 1.0;
			waypoint.color.b = 0.0;
			waypointVec.push_back(waypoint);
			++waypointCount;
		}
		this->waypointMsg_.markers = waypointVec;
	}

	void polyTrajOctomap::updateCorridorVisMsg(const std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, const std::vector<double>& corridorSizeVec){
		std::vector<visualization_msgs::Marker> corridorVec;
		visualization_msgs::Marker box;
		int boxCount = 0;
		for (int i=0; i<this->path_.size()-1; ++i){
			std::unordered_map<double, trajPlanner::pose> timeToPose = segToTimePose[i];
			for (auto itr: timeToPose){
				trajPlanner::pose p = itr.second;
				box.header.frame_id = "map";
				box.header.stamp = ros::Time();
				box.ns = "corridor";
				box.id = boxCount;
				box.type = visualization_msgs::Marker::CUBE;
				box.action = visualization_msgs::Marker::ADD;
				box.pose.position.x = p.x;
				box.pose.position.y = p.y;
				box.pose.position.z = p.z;
				box.lifetime = ros::Duration(0.5);
				box.scale.x = corridorSizeVec[i]*2;
				box.scale.y = corridorSizeVec[i]*2;
				box.scale.z = corridorSizeVec[i]*2;
				box.color.a = 0.3;
				box.color.r = 0.0;
				box.color.g = 0.5;
				box.color.b = 0.3;
				corridorVec.push_back(box);
				++boxCount;
			}
		}
		this->corridorMsg_.markers = corridorVec;
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

	void polyTrajOctomap::publishWaypoint(){
		ros::Rate r (10);
		while (ros::ok()){
			this->waypointVisPub_.publish(this->waypointMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::publishCorridor(){
		ros::Rate r (10);
		while (ros::ok()){
			this->corridorVisPub_.publish(this->corridorMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::pose2Octomap(const pose& pTraj, octomap::point3d& p){
		p.x() = pTraj.x;
		p.y() = pTraj.y;
		p.z() = pTraj.z;
	}
}
