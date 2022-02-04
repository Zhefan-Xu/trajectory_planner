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
#include <thread>
#include <mutex>
#include <unordered_map>
#include <map>

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
		std::vector<double> desiredTime_; // desired time knots based on velocity 

		OsqpEigen::Solver* xSolver_; // QP Solver
		OsqpEigen::Solver* ySolver_; 
		OsqpEigen::Solver* zSolver_;

		Eigen::VectorXd xSol_;
		Eigen::VectorXd ySol_;
		Eigen::VectorXd zSol_;
		std::vector<trajPlanner::pose> trajectory_;

		// Constraints
		bool softConstraint_; // soft constraint for waypoints
		double scDeviation_[3]; // soft constraint deviation in x y z direction
		bool corridorConstraint_;
		double corridorRes_; // resolution for corridor constraint in terms of time (e.g. 5 corridor box per 10s, res=5/10=0.5 box/s)
		std::vector<double> corridorSizeVec_; // size of corridor for each path segment 
		std::vector<std::unordered_map<double, trajPlanner::pose>> segToTimePose_;

	public:
		std::thread xWorker_;
		std::thread yWorker_;
		std::thread zWorker_;

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
		
		// helper function: solve for x, y, z
		void solveX();
		void solveY();
		void solveZ();

		// Settings
		// soft constraint for waypoints
		void setSoftConstraint(double r); 
		void setSoftConstraint(double rx, double ry);
		void setSoftConstraint(double rx, double ry, double rz);

		// corridor constraint
		void setCorridorConstraint(const std::vector<double>& corridorSizeVec, double corridorRes);
		void setCorridorConstraint(double corridorSize, double corridorRes);
		void updateCorridorParam(); // helper function
		trajPlanner::pose& interpolatePose(const trajPlanner::pose& pStart, const trajPlanner::pose& pEnd, double startTime, double endTime, double t); // helper function

		//  get pose at time t from trajectory
		trajPlanner::pose& getPose(double t);

		// get the whole trajectory with given time interval
		void getTrajectory(std::vector<trajPlanner::pose>& trajectory, double delT);
	};
}

#endif