/*
	File: polyTrajSolver.h
	----------------------
	QP polynomial trajectory solver based on OSQP-Eigen 
	The symbol of the problem is defined as:
	Min. (1/2)x'Px + q'x
	s.t.  l <= Ax <= u
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
		int paramDim_; // dimension of parameter vector
		int constraintNum_; // number of constraints
		double desiredVel_;

		std::vector<trajPlanner::pose> path_;
		std::vector<double> desiredTime_;

		OsqpEigen::Solver* xSolver_; // QP Solver
		OsqpEigen::Solver* ySolver_; 
		OsqpEigen::Solver* zSolver_;
		bool init_;

	public:
		// default constructor
		polyTrajSolver();

		polyTrajSolver(int polyDegree, int diffDegree, double desiredVel);

		// load or update path 
		void updatePath(const std::vector<trajPlanner::pose>& path);

		// Estimated the desired time of path (avg)
		void avgTimeAllocation();

		// set up the optimzation problem
		void setUpProblem();

		// construct coefficient matrix for second order of objective
		void constructP(Eigen::SparseMatrix<double>& P);

		// construct coefficient vector for first order term of objective
		void constructQ(Eigen::VectorXd& q);

		// Inquality matrix
		void constructA(Eigen::SparseMatrix<double>& A);

		// construct equality and upper and lower bound
		void constructBound(Eigen::VectorXd& lx, Eigen::VectorXd& ly, Eigen::VectorXd& lz, 
			                Eigen::VectorXd& ux, Eigen::VectorXd& uy, Eigen::VectorXd& uz);

		
		// solve the problem
		void solve();
	};
}

#endif