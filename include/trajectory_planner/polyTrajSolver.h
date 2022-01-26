/*
	File: polyTrajSolver.h
	----------------------
	QP polynomial trajectory solver based on OSQP-Eigen 
*/ 


#ifndef POLYTRAJSOLVER_H
#define POLYTRAJSOLVER_H
#include <trajectory_planner/utils.h>
#include <OsqpEigen/OsqpEigen.h>

using std::cout;
using std::endl;

namespace trajPlanner{
	class polyTrajSolver{
	private:
		int polyDegree_;
		int diffDegree_; 

		std::vector<trajPlanner::pose> path_;

		OsqpEigen::Solver* xSolver_; // QP Solver
		OsqpEigen::Solver* ySolver_; 
		OsqpEigen::Solver* zSolver_;
		bool init_;

	public:
		// default constructor
		polyTrajSolver();

		polyTrajSolver(int polyDegree, int diffDegree);

		// load or update path 
		void updatePath(const std::vector<trajPlanner::pose>& path);

		// set up the optimzation problem
		void setUpProblem();

		// solve the problem
		void solve();
	};
}

#endif