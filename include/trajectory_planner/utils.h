/*
    File: utils.h
    -----------------
    Trajectory pose definition and miscs. 
*/ 

#ifndef TRAJECTORYPLANNERUTILS_H
#define TRAJECTORYPLANNERUTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <global_planner/Point.h>
#include <Eigen/Eigen>

    

namespace trajPlanner{   
    const double PI_const = 3.1415926;
    struct pose{
        double x;
        double y;
        double z;
        double yaw;
        pose(){
            x = 0; y = 0; z = 0; yaw = 0;
        }
        pose(double _x, double _y, double _z){
            x = _x; y = _y; z = _z; yaw = 0;
        }   

        pose(double _x, double _y, double _z, double _yaw){
            x = _x; y = _y; z = _z; yaw = _yaw;
        }
    };

    inline std::ostream &operator<<(std::ostream &os, pose& pose){
        os << "pose: (" << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << ")";
        return os;
    }


    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
    	if (yaw > PI_const){
    		yaw = yaw - 2*PI_const;
    	}
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
    	// return is [0, 2pi]
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	double roll, pitch, yaw;
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    	return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline double getPoseDistance(const pose& p1, const pose& p2){
    	return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));	

    }

    inline double getYawDistance(const pose& pStart, const pose& pTarget){
        double yaw1 = pStart.yaw;
        double yaw2 = pTarget.yaw;
        double delta = std::abs(yaw2 - yaw1);
        if (delta > PI_const){
            delta = 2 * PI_const - delta;
        }
        return delta;
    }

    inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }

    template <size_t N>
    inline void convertPointPlan(const std::vector<KDTree::Point<N>>& plan, std::vector<pose>& path){
        for (KDTree::Point<N> p: plan){
            pose pPose (p[0], p[1], p[2]);
            path.push_back(pPose);
        }
    }
    

}
#endif