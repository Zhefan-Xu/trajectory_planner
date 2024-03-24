/*
    FILE: dynamicObstacleGenerator.h
    -----------------------------
    dynamic obstacle generator header
*/


#ifndef DYNAMIC_OBSTACLE_GENERATOR_H
#define DYNAMIC_OBSTACLE_GENERATOR_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <vector>

using std::cout; using std::endl;
namespace trajPlanner{
    class obstacleGenerator{
        private:
            std::string ns_;
            std::string hint_;
            ros::NodeHandle nh_;

            ros::Publisher obstacleVisPub_;
            ros::Publisher obstacleBBoxVisPub_;
            ros::Timer obGenTimer_;
            ros::Timer visTimer_;

            double delT_;
            double xmax_, ymax_, zmax_;
            double xmin_, ymin_, zmin_;
            std::vector<Eigen::Vector3d> obstacleStartPos_;
            std::vector<Eigen::Vector3d> obstacleEndPos_;
            std::vector<Eigen::Vector3d> obstacleSize_;
            std::vector<double> obstacleRefVel_;     

            std::vector<Eigen::Vector3d> obstaclePos_;
            std::vector<Eigen::Vector3d> obstacleVel_;
        
        public:
            obstacleGenerator();
            obstacleGenerator(const ros::NodeHandle& nh);
            void initParam();
            void registerPub();
            void registerCallback();

            void obGenCB(const ros::TimerEvent&);
            void linearMotion();


            void visCB(const ros::TimerEvent&);
            void publishObstacles();
            void publishObstacleBBoxes();      

            std::vector<Eigen::Vector3d> getObstaclePos();
            std::vector<Eigen::Vector3d> getObstacleVel();
            std::vector<Eigen::Vector3d> getObstacleSize();  
    };
}
#endif