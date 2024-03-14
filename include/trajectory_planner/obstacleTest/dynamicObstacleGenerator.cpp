#include <trajectory_planner/obstacleTest/dynamicObstacleGenerator.h>
namespace trajPlanner{
    obstacleGenerator::obstacleGenerator(){}

	obstacleGenerator::obstacleGenerator(const ros::NodeHandle& nh) : nh_(nh){
        this->ns_ = "obstacle_generator";
        this->hint_ = "[Dynamic Obstacle Generator]";
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	void obstacleGenerator::initParam(){
        // dynamic obstacle for testing
        if (not this->nh_.getParam(this->ns_ + "/discrete_time_step", this->delT_)){
            this->delT_ = 0.1;
            cout << this->hint_ << ": No discrete time step param found. Use default: 0.1s." << endl;
        }
        else{
            cout<< this->hint_ <<": using discrete time step: "<<this->delT_<<endl;
        }
		std::vector<double> obstacleVecTemp;
		if (not this->nh_.getParam(this->ns_ + "/obstacles", obstacleVecTemp)){
			// this->obstacles_.clear();
            this->obstaclePos_.clear();
            this->obstacleVel_.clear();
            this->obstacleSize_.clear();
			cout << this->hint_ << ": No obstacles param found." << endl;
		}
		else{
            this->obstaclePos_.clear();
            this->obstacleVel_.clear();
            this->obstacleSize_.clear();
			int numObstacle = int(obstacleVecTemp.size())/9;
			if (numObstacle != 0){
				for (int i=0; i<numObstacle; ++i){
					Eigen::Vector3d pos;
					Eigen::Vector3d vel;
					Eigen::Vector3d size;
					pos << obstacleVecTemp[i*9 + 0],obstacleVecTemp[i*9 + 1],obstacleVecTemp[i*9 + 2]-obstacleVecTemp[i*9 + 5]/2;
					size << obstacleVecTemp[i*9 + 3],obstacleVecTemp[i*9 + 4],obstacleVecTemp[i*9 + 5];
					vel << obstacleVecTemp[i*9 + 6],obstacleVecTemp[i*9 + 7],obstacleVecTemp[i*9 + 8];
					this->obstaclePos_.push_back(pos);
                    this->obstacleVel_.push_back(vel);
                    this->obstacleSize_.push_back(size);	
				}
			}
            cout << this->hint_ <<": number of obstacle: " << numObstacle << endl; 
		}
        std::vector<double> maxVecTemp;
        if (not this->nh_.getParam(this->ns_ + "/max", maxVecTemp)){
            this->xmax_ = 10;
            this->ymax_ = 10;
            this->zmax_ = 10;
            cout << this->hint_ << ": No max xyz param found. Use default: [10,10,10]" << endl;
        }
        else{
            this->xmax_ = maxVecTemp[0];
            this->ymax_ = maxVecTemp[1];
            this->zmax_ = maxVecTemp[2];
            cout<< this->hint_ <<": max x: "<<this->xmax_<<", max y: "<<this->ymax_<<", max z: "<<this->zmax_<<endl;
        }
        std::vector<double> minVecTemp;
        if (not this->nh_.getParam(this->ns_ + "/min", minVecTemp)){
            this->xmin_ = -10;
            this->ymin_ = -10;
            this->zmin_ = -10;
            cout << this->hint_ << ": No min xyz param found. Use default: [-10,-10,-10]" << endl;
        }
        else{
            this->xmin_ = minVecTemp[0];
            this->ymin_ = minVecTemp[1];
            this->zmin_ = minVecTemp[2];
            cout<< this->hint_ <<": min x: "<<this->xmin_<<", min y: "<<this->ymin_<<", min z: "<<this->zmin_<<endl;
        }
    }

    void obstacleGenerator::registerPub(){
        this->obstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/obstacle", 1000);
    }

    void obstacleGenerator::registerCallback(){
        this->obGenTimer_ = this->nh_.createTimer(ros::Duration(this->delT_), &obstacleGenerator::obGenCB, this);
        this->visTimer_ = this->nh_.createTimer(ros::Duration(this->delT_), &obstacleGenerator::visCB, this);
    }

    void obstacleGenerator::obGenCB(const ros::TimerEvent&){
        for (int i = 0; i < this->obstaclePos_.size(); i++){
            this->linearMotion(this->obstaclePos_[i],this->obstacleVel_[i]);
        }   
    }

    void obstacleGenerator::linearMotion(Eigen::Vector3d &pos, Eigen::Vector3d &vel){
        if (pos(0) >= this->xmax_ || pos(0) <= this->xmin_){
            vel(0) = -vel(0);
        }
        if (pos(1) >= this->ymax_ || pos(1) <= this->ymin_){
            vel(1) = -vel(1);
        }
        if (pos(2) >= this->zmax_ || pos(2) <= this->zmin_){
            vel(2) = -vel(2);
        }
        pos(0) = pos(0) + vel(0)*this->delT_;
        pos(1) = pos(1) + vel(1)*this->delT_;
        pos(2) = pos(2) + vel(2)*this->delT_;
    }

    void obstacleGenerator::visCB(const ros::TimerEvent&){
        this->publishObstacles();
    }

    void obstacleGenerator::publishObstacles(){
        visualization_msgs::MarkerArray msg;
        std::vector<visualization_msgs::Marker> markers;
        int i = 0;
        for (int i = 0; i < this->obstaclePos_.size(); i++){
            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = ros::Time();
            m.ns = "obstacles";
            m.id = 99999 + i;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = this->obstaclePos_[i](0);
            m.pose.position.y = this->obstaclePos_[i](1);
            m.pose.position.z = this->obstaclePos_[i](2)+this->obstacleSize_[i](2)/2;
            m.pose.orientation.x = 0;
            m.pose.orientation.y = 0;
            m.pose.orientation.z = 0;
            m.pose.orientation.w = 1;
            m.scale.x = this->obstacleSize_[i](0);
            m.scale.y = this->obstacleSize_[i](1);
            m.scale.z = this->obstacleSize_[i](2);
            m.color.a = 1.0;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.lifetime = ros::Duration(0.5);
            markers.push_back(m);
        }
        // cout << markers.size() << endl;
        msg.markers = markers;
        this->obstacleVisPub_.publish(msg);
	}

    std::vector<Eigen::Vector3d> obstacleGenerator::getObstaclePos(){
        return this->obstaclePos_;
    }

    std::vector<Eigen::Vector3d> obstacleGenerator::getObstacleVel(){
        return this->obstacleVel_;
    }

    std::vector<Eigen::Vector3d> obstacleGenerator::getObstacleSize(){
        return this->obstacleSize_;
    }
}
