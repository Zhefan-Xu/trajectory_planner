#ifndef TRAJECTORYPLANNERUTILS_H
#define TRAJECTORYPLANNERUTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <global_planner/Point.h>

#define PI_const 3.1415926
    

namespace trajPlanner{
    using namespace std;
    
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

    struct obstacle{
    	double x, y, z;
    	double xsize, ysize, zsize;
    	double vx, vy, vz;
    	double varX, varY, varZ;
    	double varVx, varVy, varVz;
    };


    geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
    	if (yaw > PI_const){
    		yaw = yaw - 2*PI_const;
    	}
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    double rpy_from_quaternion(geometry_msgs::Quaternion quat){
    	// return is [0, 2pi]
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	double roll, pitch, yaw;
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    	return yaw;
    }

    void rpy_from_quaternion(geometry_msgs::Quaternion quat, double &roll, double &pitch, double &yaw){
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    double getDistance(pose p1, pose p2){
    	return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));	

    }

    double getDistance(obstacle p1, pose p2){
        return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));    

    }

    double getDistance(pose p1, obstacle p2){
        return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));    

    }


    float my_logf (float);

    /* compute inverse error functions with maximum error of 2.35793 ulp */
    float my_erfinvf (float a)
    {
        float p, r, t;
        t = fmaf (a, 0.0f - a, 1.0f);
        t = my_logf (t);
        if (fabsf(t) > 6.125f) { // maximum ulp error = 2.35793
            p =              3.03697567e-10f; //  0x1.4deb44p-32 
            p = fmaf (p, t,  2.93243101e-8f); //  0x1.f7c9aep-26 
            p = fmaf (p, t,  1.22150334e-6f); //  0x1.47e512p-20 
            p = fmaf (p, t,  2.84108955e-5f); //  0x1.dca7dep-16 
            p = fmaf (p, t,  3.93552968e-4f); //  0x1.9cab92p-12 
            p = fmaf (p, t,  3.02698812e-3f); //  0x1.8cc0dep-9 
            p = fmaf (p, t,  4.83185798e-3f); //  0x1.3ca920p-8 
            p = fmaf (p, t, -2.64646143e-1f); // -0x1.0eff66p-2 
            p = fmaf (p, t,  8.40016484e-1f); //  0x1.ae16a4p-1 
        } else { // maximum ulp error = 2.35002
            p =              5.43877832e-9f;  //  0x1.75c000p-28 
            p = fmaf (p, t,  1.43285448e-7f); //  0x1.33b402p-23 
            p = fmaf (p, t,  1.22774793e-6f); //  0x1.499232p-20 
            p = fmaf (p, t,  1.12963626e-7f); //  0x1.e52cd2p-24 
            p = fmaf (p, t, -5.61530760e-5f); // -0x1.d70bd0p-15 
            p = fmaf (p, t, -1.47697632e-4f); // -0x1.35be90p-13 
            p = fmaf (p, t,  2.31468678e-3f); //  0x1.2f6400p-9 
            p = fmaf (p, t,  1.15392581e-2f); //  0x1.7a1e50p-7 
            p = fmaf (p, t, -2.32015476e-1f); // -0x1.db2aeep-3 
            p = fmaf (p, t,  8.86226892e-1f); //  0x1.c5bf88p-1 
        }
        r = a * p;
        return r;
    }

    /* compute natural logarithm with a maximum error of 0.85089 ulp */
    float my_logf (float a)
    {
        float i, m, r, s, t;
        int e;

        m = frexpf (a, &e);
        if (m < 0.666666667f) { // 0x1.555556p-1
            m = m + m;
            e = e - 1;
        }
        i = (float)e;
        /* m in [2/3, 4/3] */
        m = m - 1.0f;
        s = m * m;
        /* Compute log1p(m) for m in [-1/3, 1/3] */
        r =             -0.130310059f;  // -0x1.0ae000p-3
        t =              0.140869141f;  //  0x1.208000p-3
        r = fmaf (r, s, -0.121484190f); // -0x1.f19968p-4
        t = fmaf (t, s,  0.139814854f); //  0x1.1e5740p-3
        r = fmaf (r, s, -0.166846052f); // -0x1.55b362p-3
        t = fmaf (t, s,  0.200120345f); //  0x1.99d8b2p-3
        r = fmaf (r, s, -0.249996200f); // -0x1.fffe02p-3
        r = fmaf (t, m, r);
        r = fmaf (r, m,  0.333331972f); //  0x1.5554fap-2
        r = fmaf (r, m, -0.500000000f); // -0x1.000000p-1
        r = fmaf (r, s, m);
        r = fmaf (i,  0.693147182f, r); //  0x1.62e430p-1 // log(2)
        if (!((a > 0.0f) && (a <= 3.40282346e+38f))) { // 0x1.fffffep+127
            r = a + a;  // silence NaNs if necessary
            if (a  < 0.0f) r = ( 0.0f / 0.0f); //  NaN
            if (a == 0.0f) r = (-1.0f / 0.0f); // -Inf
        }
        return r;
    }


    template <size_t N>
    void convertPointPlan(const std::vector<KDTree::Point<N>>& plan, std::vector<pose>& path){
        for (KDTree::Point<N> p: plan){
            pose pPose (p[0], p[1], p[2]);
            path.push_back(pPose);
        }
    }

}
#endif