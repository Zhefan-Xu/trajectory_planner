/*
	FILE: bsplineTraj.cpp
	-----------------------------
	bspline trajectory solver implementation based on occupancy grid map 
*/
#include <trajectory_planner/bsplineTraj.h>

namespace trajPlanner{
	bsplineTraj::bsplineTraj(){}

	void bsplineTraj::init(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	void bsplineTraj::initParam(){
		if (not this->nh_.getParam("bspline_traj/timestep", this->ts_)){
			this->ts_ = 0.1;
			cout << "[BsplineTraj]" << ": No timestep. Use default: 0.1" << endl;
		}
		else{
			cout << "[BsplineTraj]" << ": timestep: " << this->ts_ << endl;
		}
	}

	void bsplineTraj::registerPub(){
		this->controlPointsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("bspline_traj/control_points", 10);
		this->currTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("bspline_traj/trajectory", 10);
	}

	void bsplineTraj::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &bsplineTraj::visCB, this);
	}

	void bsplineTraj::updatePath(const nav_msgs::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition){
		Eigen::MatrixXd controlPoints;
		std::vector<Eigen::Vector3d> curveFitPoints;
		this->pathMsgToEigenPoints(path, curveFitPoints);
		this->bspline_.parameterizeToBspline(this->ts_, curveFitPoints, startEndCondition, controlPoints);
		this->optData_.controlPoints = controlPoints;
		this->init_ = true;
	}

	std::vector<Eigen::Vector3d> bsplineTraj::evalTraj(){
		this->bspline_ = trajPlanner::bspline (bsplineDegree, this->optData_.controlPoints, this->ts_);
		std::vector<Eigen::Vector3d> traj;
		Eigen::Vector3d p;
		for (double t=0; t<=this->bspline_.getDuration(); t+=this->ts_){
			p = this->bspline_.at(t);
			traj.push_back(p);
		}
		return traj;
	}	

	nav_msgs::Path bsplineTraj::evalTrajToMsg(){
		std::vector<Eigen::Vector3d> trajTemp = this->evalTraj();
		nav_msgs::Path traj;
		this->eigenPointsToPathMsg(trajTemp, traj);
		return traj;
	}


	void bsplineTraj::visCB(const ros::TimerEvent& ){
		if (this->init_){
			this->publishControlPoints();
			this->publishCurrTraj();
		}
	}

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
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.3;
			point.scale.y = 0.3;
			point.scale.z = 0.3;
			point.color.a = 0.5;
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