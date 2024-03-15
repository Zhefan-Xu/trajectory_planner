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
            ros::Timer obGenTimer_;
            ros::Timer visTimer_;

            double delT_;
            double xmax_, ymax_, zmax_;
            double xmin_, ymin_, zmin_;
            std::vector<Eigen::Vector3d> obstaclePos_;
            std::vector<Eigen::Vector3d> obstacleVel_;
            std::vector<Eigen::Vector3d> obstacleSize_;
        
        public:
            obstacleGenerator();
            obstacleGenerator(const ros::NodeHandle& nh);
            void initParam();
            void registerPub();
            void registerCallback();

            void obGenCB(const ros::TimerEvent&);
            void linearMotion(Eigen::Vector3d &pos, Eigen::Vector3d &vel);
            std::vector<Eigen::Vector3d> getObstaclePos();
            std::vector<Eigen::Vector3d> getObstacleVel();
            std::vector<Eigen::Vector3d> getObstacleSize();

            void visCB(const ros::TimerEvent&);
            void publishObstacles();        
    };
}
#endif