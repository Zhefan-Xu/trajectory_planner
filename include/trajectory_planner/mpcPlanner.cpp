/*
	FILE: mpcPlanner.cpp
	-----------------------------
	mpc trajectory solver implementation based on occupancy grid map 
*/

#include <trajectory_planner/mpcPlanner.h>

namespace trajPlanner{
	mpcPlanner::mpcPlanner(const ros::NodeHandle& nh) : nh_(nh){}

	void mpcPlanner::makePlan(){
		DifferentialState x;
		DifferentialState y;
		DifferentialState z;
		DifferentialState vx;
		DifferentialState vy;
		DifferentialState vz;	

		//Control Input
		Control ax;
		Control ay;
		Control az;
		
		DifferentialEquation f;
		f << dot(x) == vx;
		f << dot(y) == vy;
		f << dot(z) == vz;
		f << dot(vx) == ax; 
		f << dot(vy) == ay;
		f << dot(vz) == az;
		

	}
}