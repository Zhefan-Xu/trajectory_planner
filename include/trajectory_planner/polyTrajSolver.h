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
#include <geometry_msgs/Twist.h>
// #include <OsqpEigen/OsqpEigen.h>
#include <trajectory_planner/third_party/OsqpEigen/OsqpEigen.h>
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
		int continuityDegree_;
		int paramDim_; // dimension of parameter vector
		int constraintNum_; // number of constraints
		double desiredVel_;
		double desiredAcc_;
		bool init_;
		
		std::vector<trajPlanner::pose> path_;
		std::vector<double> desiredTime_; // desired time knots based on velocity 
		geometry_msgs::Twist initVel_;
		geometry_msgs::Twist endVel_;
		geometry_msgs::Twist initAcc_;
		geometry_msgs::Twist endAcc_;

		std::shared_ptr<OsqpEigen::Solver> xSolver_; // QP Solver
		std::shared_ptr<OsqpEigen::Solver> ySolver_; 
		std::shared_ptr<OsqpEigen::Solver> zSolver_;

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

		polyTrajSolver(int polyDegree, int diffDegree, int continuityDegree, double desiredVel);
		polyTrajSolver(int polyDegree, int diffDegree, int continuityDegree, double desiredVel, double desiredAcc);

		// load or update path 
		void updatePath(const std::vector<trajPlanner::pose>& path);
		void updateInitVel(double vx, double vy, double vz);
		void updateInitVel(const geometry_msgs::Twist& v);
		void updateEndVel(double vx, double vy, double vz);
		void updateEndVel(const geometry_msgs::Twist& v);
		void updateInitAcc(double ax, double ay, double az);
		void updateInitAcc(const geometry_msgs::Twist& a);
		void updateEndAcc(double ax, double ay, double az);
		void updateEndAcc(const geometry_msgs::Twist& a);
		void setDefaultInit();

		// Estimated the desired time of path (avg)
		void avgTimeAllocation();
		void equalTimeAllocation();

		// set up the optimzation problem
		int getConstraintNum();
		void setUpProblem();
		void updateProblem(); // update constraint

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


		// time allocation reevaluaton
		void evalTrajectory();



		// Settings
		// soft constraint for waypoints
		void setSoftConstraint(double r); 
		void setSoftConstraint(double rx, double ry);
		void setSoftConstraint(double rx, double ry, double rz);

		// corridor constraint
		void setCorridorConstraint(const std::vector<double>& corridorSizeVec, double corridorRes);
		void setCorridorConstraint(double corridorSize, double corridorRes);
		void updateCorridorParam(); // helper function
		trajPlanner::pose interpolatePose(const trajPlanner::pose& pStart, const trajPlanner::pose& pEnd, double startTime, double endTime, double t); // helper function

		//  get pose at time t from trajectory
		trajPlanner::pose getPose(double t);
		Eigen::Vector3d getPos(double t);
		Eigen::Vector3d getVel(double t);
		Eigen::Vector3d getAcc(double t);

		// get the whole trajectory with given time interval
		void getTrajectory(std::vector<trajPlanner::pose>& trajectory, double delT);

		// get time knot
		std::vector<double>& getTimeKnot();

		//get corridor for visualization:
		void getCorridor(std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, std::vector<double>& corridorSizeVec);
	};
}

#endif