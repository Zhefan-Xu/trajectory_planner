/*
	File: polyTrajSolver.cpp
	------------------------
	function definition of polynomial trajectory solver

*/ 
#include <trajectory_planner/polyTrajSolver.h>

namespace trajPlanner{
	polyTrajSolver::polyTrajSolver(){}

	polyTrajSolver::polyTrajSolver(int polyDegree, int diffDegree, double desiredVel)
	: polyDegree_(polyDegree), diffDegree_(diffDegree), desiredVel_(desiredVel){
		this->xSolver_ = new OsqpEigen::Solver ();
		this->ySolver_ = new OsqpEigen::Solver ();
		this->zSolver_ = new OsqpEigen::Solver ();
		this->init_ = false;
	}
	

	void polyTrajSolver::updatePath(const std::vector<trajPlanner::pose>& path){
		this->path_ = path;

		int pathSegNum = this->path_.size()-1;
		this->paramDim_ = (this->polyDegree_+1) * pathSegNum;
		this->constraintNum_ = (2+ pathSegNum-1 + pathSegNum-1) + (2+pathSegNum-1) + (2+pathSegNum-1) + (pathSegNum-1) * (this->diffDegree_-2); // position, velocity, acceleration, jerk, snap
		this->avgTimeAllocation();
	}

	void polyTrajSolver::avgTimeAllocation(){
		// this->desiredTime_.clear();
		double totalTime = 0;
		this->desiredTime_.push_back(totalTime);
		double distance;
		for (int i=0; i<this->path_.size(); ++i){
			if (i != 0){
				distance = trajPlanner::getPoseDistance(this->path_[i], this->path_[i-1]);
				totalTime += (double) distance/this->desiredVel_;
				this->desiredTime_.push_back(totalTime);
			}
		}
	}

	void polyTrajSolver::setUpProblem(){
		// set three solvers' argument
		Eigen::SparseMatrix<double> P; // Hessian matrix
		this->constructP(P); 

		Eigen::VectorXd q;
		this->constructQ(q); // gradient vector (first order term)

		Eigen::SparseMatrix<double> A; // Equality and inqualit matrix
		this->constructA(A);

		Eigen::VectorXd lx, ly, lz; // lower bound
		Eigen::VectorXd ux, uy, uz; // uppper bound
		this->constructBound(lx, ly, lz, ux, uy, uz);


		// Initialize three solver for x y z
		// x solver:
		this->xSolver_->data()->setNumberOfVariables(this->paramDim_);
		this->xSolver_->data()->setNumberOfConstraints(this->constraintNum_);
		if(!this->xSolver_->data()->setHessianMatrix(P)) return;
    	if(!this->xSolver_->data()->setGradient(q)) return;
    	if(!this->xSolver_->data()->setLinearConstraintsMatrix(A)) return;
    	if(!this->xSolver_->data()->setLowerBound(lx)) return;
    	if(!this->xSolver_->data()->setUpperBound(ux)) return;
    	if(!this->xSolver_->initSolver()) return;

    	// y solver:
    	this->ySolver_->data()->setNumberOfVariables(this->paramDim_);
    	this->ySolver_->data()->setNumberOfConstraints(this->constraintNum_);
		if(!this->ySolver_->data()->setHessianMatrix(P)) return;
    	if(!this->ySolver_->data()->setGradient(q)) return;
    	if(!this->ySolver_->data()->setLinearConstraintsMatrix(A)) return;
    	if(!this->ySolver_->data()->setLowerBound(ly)) return;
    	if(!this->ySolver_->data()->setUpperBound(uy)) return;
    	if(!this->ySolver_->initSolver()) return;

    	// z solver:
    	this->zSolver_->data()->setNumberOfVariables(this->paramDim_);
		this->zSolver_->data()->setNumberOfConstraints(this->constraintNum_);
		if(!this->zSolver_->data()->setHessianMatrix(P)) return;
    	if(!this->zSolver_->data()->setGradient(q)) return;
    	if(!this->zSolver_->data()->setLinearConstraintsMatrix(A)) return;
    	if(!this->zSolver_->data()->setLowerBound(lz)) return;
    	if(!this->zSolver_->data()->setUpperBound(uz)) return;
		if(!this->zSolver_->initSolver()) return;
		
		this->init_ = true;
	}

	void polyTrajSolver::constructP(Eigen::SparseMatrix<double>& P){
		// coefficient matrix for second order of objective
		Eigen::VectorXd coeff; // coefficient factor for optimization parameters
		coeff.resize(this->polyDegree_+1);
		Eigen::MatrixXd m; // temp matrix for store
		P.resize(this->paramDim_, this->paramDim_);

		// assign parameters for each path segment
		for (int i=0; i<this->path_.size()-1; ++i){
			// go through each degree order from 0 to polynomial degree
			for (int d=0; d<this->polyDegree_+1; ++d){
				if (d < this->diffDegree_){
					coeff(d) = 0;
				}
				else{
					double factor = 1.0;
					for (int j=0; j<this->diffDegree_-1; ++j){
						factor *= (d-j);
					}
					factor *= (pow(this->desiredTime_[i+1], d-this->diffDegree_+1) - pow(this->desiredTime_[i], d-this->diffDegree_+1));
					coeff(d) = factor;
				}
			}
			m = coeff * coeff.transpose();

			int startIdx = (this->polyDegree_+1) * i;
			for (int row=0; row<m.rows(); ++row){
				for (int col=0; col<m.cols(); ++col){
					double value = m(row, col);
					if (value != 0){
						P.insert(startIdx+row, startIdx+col) = value;
					}
				}
			}
		}
	}

	void polyTrajSolver::constructQ(Eigen::VectorXd& q){
		// coefficient vector for first order term of objective
		q = Eigen::VectorXd::Zero(this->paramDim_);
	}

	void polyTrajSolver::constructA(Eigen::SparseMatrix<double>& A){
		// equality and inequality constraint matrix
		A.resize(this->constraintNum_, this->paramDim_);

		int countConstraint = 0;

		// Position Constraint: 2 endpoint, k-1 midpoint, k-1 continuity (K is the number of path segments)
		{
			// =====================2 Endpoint==============================
			{
				int startIdx = 0;
				double startTime = 0;
				for (int d=0; d<this->polyDegree_+1; ++d){ // start position
				 	double factor = pow(startTime, d);
				 	if (factor != 0){
				 		A.insert(countConstraint, startIdx+d) = factor;
					}
				}
				++countConstraint;

				int endIdx = (this->path_.size()-2) * (this->polyDegree_+1);
				double endTime = this->desiredTime_[this->desiredTime_.size()-1];
				for (int d=0; d<this->polyDegree_+1; ++d){ // end position
					double factor = pow(endTime, d);
					if (factor != 0){
						A.insert(countConstraint, endIdx+d) = factor;
					}
				}
				++countConstraint;
			}

			cout << A << endl;
			cout << "Number of constraints: " << countConstraint << endl;

			// ==================K-1 midpoint=============================
			{
				// Note: midpoint is only for first k-1 path segment, it is the right point on the path segment
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int currStartIdx = (this->polyDegree_+1) * i;
					for (int d=0; d<this->polyDegree_+1; ++d){
						double factor = pow(currTime, d);
						if (factor != 0){
							A.insert(countConstraint, currStartIdx+d) = factor;
						}
					}
					++countConstraint;
				}
			}


			cout << "k-1 = " << this->path_.size() - 2 << endl;
			cout << A << endl;
			cout << "Number of constraints: " << countConstraint << endl;

			// ==================K-1 Continuity===========================
			{
				// from "LEFT" t0 "RIGHT"
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						double factor = pow(currTime, d);
						if (factor != 0){
							A.insert(countConstraint, leftStartIdx+d) = factor;
							A.insert(countConstraint, rightStartIdx+d) = -factor;
						}
					}
					++countConstraint;
				}	
			}

			cout << "k-1 = " << this->path_.size() - 2 << endl;
			cout << A << endl;
			cout << "Number of constraints: " << countConstraint << endl;
		}


		// Velcity Constraint:
		{
			// ================2 Endpoint===============================
			{
				// Note velocity = 0
				double startIdx = 0;
				double startTime = this->desiredTime_[0];
				for (int d=0; d<this->polyDegree_+1; ++d){ // start index
					if (d != 0){
						double factor = d * pow(startTime, d-1);
						if (factor != 0){
							A.insert(countConstraint, startIdx+d) = factor;
						}
					}
				}
				++countConstraint;

				int endIdx = (this->path_.size()-2) * (this->polyDegree_+1);
				double endTime = this->desiredTime_[this->desiredTime_.size()-1];
				for (int d=0; d<this->polyDegree_+1; ++d){
					if (d != 0){
						double factor = d * pow(endTime, d-1);
						if (factor != 0){
							A.insert(countConstraint, endIdx+d) = factor;
						}
					}
				}
				++countConstraint;

				cout << A << endl;
				cout << "Number of constraints: " << countConstraint << endl;
			}


			// ================K-1 Continuity===========================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d != 0){ // C0 index is alawys zero
							double factor = d * pow(currTime, d-1);
							if (factor != 0){
								A.insert(countConstraint, leftStartIdx+d) = factor;
								A.insert(countConstraint, rightStartIdx+d) = -factor;
							}
						}
					}
					++countConstraint;
				}
				cout << "k-1 = " << this->path_.size() - 2 << endl;
				cout << A << endl;
				cout << "Number of constraints: " << countConstraint << endl;
			}
		}

		// Acceleration Constraint:
		{
			// ===============2 Endpoint================================
			{
				// Note: acceleration = 0
				int startIdx = 0;
				double startTime = this->desiredTime_[0];
				for (int d=0; d<this->polyDegree_+1; ++d){
					if (d > 1){
						double factor = d * (d-1) * pow(startTime, d-2);
						if (factor != 0){
							A.insert(countConstraint, startIdx+d) = factor;
						}
					}
				}
				++countConstraint;

				int endIdx = (this->path_.size()-2) * (this->polyDegree_+1);
				double endTime = this->desiredTime_[this->desiredTime_.size()-1];
				for (int d=0; d<this->polyDegree_+1; ++d){
					if (d > 1){
						double factor = d * (d-1) * pow(endTime, d-2);
						if (factor != 0){
							A.insert(countConstraint, endIdx+d) = factor;
						}
					}
				}
				++countConstraint;
				cout << A << endl;
				cout << "Number of constraints: " << countConstraint << endl;
			}

			// ===============K-1 Continuity============================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d > 1){
							double factor = d * (d-1) * pow(currTime, d-2);
							if (factor != 0){
								A.insert(countConstraint, leftStartIdx+d) = factor;
								A.insert(countConstraint, rightStartIdx+d) = -factor;
							}
						}
					}
					++countConstraint;
				}
				cout << "k-1 = " << this->path_.size() - 2 << endl;
				cout << A << endl;
				cout << "Number of constraints: " << countConstraint << endl;
			}
		}

		// Higher order constraint: jerk snap
		{
			for (int diffD=3; diffD<=this->diffDegree_+1; ++diffD){

			}
		}

	}

	void polyTrajSolver::constructBound(Eigen::VectorXd& lx, Eigen::VectorXd& ly, Eigen::VectorXd& lz, Eigen::VectorXd& ux, Eigen::VectorXd& uy, Eigen::VectorXd& uz){
		// lower and upper bounds for equality and inequality
	}


	void polyTrajSolver::solve(){
		if (not this->init_){
			this->setUpProblem();
		}
	}
}