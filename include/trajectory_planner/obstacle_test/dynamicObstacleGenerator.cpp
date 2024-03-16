/*
    FILE: dynamicObstacleGenerator.cpp
    -----------------------------
    dynamic obstacle generator function implementation
*/

#include <trajectory_planner/obstacle_test/dynamicObstacleGenerator.h>
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
            cout<< this->hint_ <<": Discrete time step: "<<this->delT_<<endl;
        }
		// std::vector<double> obstacleVecTemp;
		// if (not this->nh_.getParam(this->ns_ + "/obstacles", obstacleVecTemp)){
        //     this->obstaclePos_.clear();
        //     this->obstacleVel_.clear();
        //     this->obstacleSize_.clear();
		// 	cout << this->hint_ << ": No obstacles param found." << endl;
		// }
		// else{
        //     this->obstaclePos_.clear();
        //     this->obstacleVel_.clear();
        //     this->obstacleSize_.clear();
		// 	int numObstacle = int(obstacleVecTemp.size())/9;
		// 	if (numObstacle != 0){
		// 		for (int i=0; i<numObstacle; ++i){
		// 			Eigen::Vector3d pos;
		// 			Eigen::Vector3d vel;
		// 			Eigen::Vector3d size;
		// 			pos << obstacleVecTemp[i*9 + 0], obstacleVecTemp[i*9 + 1], obstacleVecTemp[i*9 + 2]-obstacleVecTemp[i*9 + 5]/2;
		// 			size << obstacleVecTemp[i*9 + 3], obstacleVecTemp[i*9 + 4], obstacleVecTemp[i*9 + 5];
		// 			vel << obstacleVecTemp[i*9 + 6], obstacleVecTemp[i*9 + 7], obstacleVecTemp[i*9 + 8];
		// 			this->obstaclePos_.push_back(pos);
        //             this->obstacleVel_.push_back(vel);
        //             this->obstacleSize_.push_back(size);	
		// 		}
		// 	}
        //     cout << this->hint_ <<": number of obstacle: " << numObstacle << endl; 
		// }
        std::vector<double> obstacleParamTemp;
		if (not this->nh_.getParam(this->ns_ + "/dynamic_obstacles", obstacleParamTemp)){
            this->obstacleStartPos_.clear();
            this->obstacleEndPos_.clear();
            this->obstacleRefVel_.clear();
            this->obstacleSize_.clear();
			cout << this->hint_ << ": No dynamic obstacles param found." << endl;
		}
		else{
            this->obstacleStartPos_.clear();
            this->obstacleEndPos_.clear();
            this->obstacleRefVel_.clear();
            this->obstacleSize_.clear();
			int numObstacle = int(obstacleParamTemp.size())/10;
			if (numObstacle != 0){
				for (int i=0; i<numObstacle; ++i){
					Eigen::Vector3d startPos;
					Eigen::Vector3d endPos;
                    double refVel;
					Eigen::Vector3d size;
					startPos << obstacleParamTemp[i*10+0], obstacleParamTemp[i*10+1], obstacleParamTemp[i*10+2];
					endPos << obstacleParamTemp[i*10+3], obstacleParamTemp[i*10+4], obstacleParamTemp[i*10+5];
                    refVel = obstacleParamTemp[i*10+6];
					size << obstacleParamTemp[i*10+7], obstacleParamTemp[i*10+8], obstacleParamTemp[i*10+9];
                    this->obstacleStartPos_.push_back(startPos);
                    this->obstacleEndPos_.push_back(endPos);
                    this->obstacleRefVel_.push_back(refVel);
                    this->obstacleSize_.push_back(size);	
				}
                this->obstaclePos_ = this->obstacleStartPos_;
			}
            cout << this->hint_ <<": number of dynamic obstacle: " << numObstacle << endl; 
		}
    }

    void obstacleGenerator::registerPub(){
        this->obstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/obstacle", 1000);
        this->obstacleBBoxVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/obstacle_bbox", 1000);
    }

    void obstacleGenerator::registerCallback(){
        this->obGenTimer_ = this->nh_.createTimer(ros::Duration(this->delT_), &obstacleGenerator::obGenCB, this);
        this->visTimer_ = this->nh_.createTimer(ros::Duration(this->delT_), &obstacleGenerator::visCB, this);
    }

    void obstacleGenerator::obGenCB(const ros::TimerEvent&){
        this->linearMotion();
    }

    void obstacleGenerator::linearMotion(){
        this->obstacleVel_.clear();
        for (int i=0; i<this->obstacleSize_.size();i++){            
            Eigen::Vector3d startPos = this->obstacleStartPos_[i];
            Eigen::Vector3d endPos = this->obstacleEndPos_[i];
            double refVel = this->obstacleRefVel_[i];
            Eigen::Vector3d currPos = this->obstaclePos_[i];
            Eigen::Vector3d currVel = (endPos-startPos).normalized()*refVel;
            if ((currPos-endPos).norm()<=0.1){
                this->obstacleStartPos_[i] = endPos;
                this->obstacleEndPos_[i] = startPos;
            }
            currPos += (endPos-startPos).normalized()*refVel*this->delT_;
            this->obstaclePos_[i] = currPos;
            this->obstacleVel_.push_back(currVel);
        }        
    }

    void obstacleGenerator::visCB(const ros::TimerEvent&){
        this->publishObstacles();
        this->publishObstacleBBoxes();
    }

    void obstacleGenerator::publishObstacles(){
        visualization_msgs::MarkerArray msg;
        std::vector<visualization_msgs::Marker> markers;
        int i = 0;
        for (int i = 0; i < int(this->obstaclePos_.size()); i++){
            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = ros::Time();
            m.ns = "obstacles_ellipsoid";
            m.id = i;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = this->obstaclePos_[i](0);
            m.pose.position.y = this->obstaclePos_[i](1);
            m.pose.position.z = this->obstaclePos_[i](2);
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
        msg.markers = markers;
        this->obstacleVisPub_.publish(msg);
	}

    void obstacleGenerator::publishObstacleBBoxes(){
        visualization_msgs::Marker line;
        visualization_msgs::MarkerArray lines;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "dynamic_obstacles_bbox";  
        line.scale.x = 0.06;
        line.color.r = 0;
        line.color.g = 0;
        line.color.b = 1;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(0.1);
        
        for(int i = 0; i < int(this->obstaclePos_.size()); i++){
            // visualization msgs
            double x = this->obstaclePos_[i](0); 
            double y = this->obstaclePos_[i](1); 
            double z = (this->obstaclePos_[i](2) + this->obstacleSize_[i](2)/2)/2; 

            // double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
            // double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
            double x_width = this->obstacleSize_[i](0);
            double y_width = this->obstacleSize_[i](1);
            double z_width = 2*z;

            // double z = 
            
            std::vector<geometry_msgs::Point> verts;
            geometry_msgs::Point p;
            // vertice 0
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 1
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 2
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 3
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 4
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 5
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 6
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 7
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);
            
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
            
            for (size_t i=0;i<12;i++){
                line.points.push_back(verts[vert_idx[i][0]]);
                line.points.push_back(verts[vert_idx[i][1]]);
            }
            
            lines.markers.push_back(line);
            
            line.id++;
        }
        this->obstacleBBoxVisPub_.publish(lines);        
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
