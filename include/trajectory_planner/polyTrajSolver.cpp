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
			}
		}

		// Higher order constraint: jerk snap
		{
			//=================K-1 Continuity in Jerk===================
			if (this->diffDegree_ >= 3){
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d > 2){
							double factor = d * (d-1) * (d-2) * pow(currTime, d-3);
							if (factor != 0){
								A.insert(countConstraint, leftStartIdx+d) = factor;
								A.insert(countConstraint, rightStartIdx+d) = -factor;
							}
						}
					}
					++countConstraint;
				}
			}

			//=================K-1 Conitnuity in Snap===================
			if (this->diffDegree_ >= 4){
				for (int i=0; i<this->path_.size()-2; ++i){
					double currTime = this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d>3){
							double factor = d * (d-1) * (d-2) * (d-3) * pow(currTime, d-4);
							if (factor != 0){
								A.insert(countConstraint, leftStartIdx+d) = factor;
								A.insert(countConstraint, rightStartIdx+d) = -factor;
							}
						}
					}
					++countConstraint;
				}
			}
			// cout << "k-1 = " << this->path_.size() - 2 << endl;
			// cout << A << endl;
			cout << "Number of constraints: " << countConstraint << endl;
			cout << "Expected constraints: " << this->constraintNum_ << endl;
		}
	}

	void polyTrajSolver::constructBound(Eigen::VectorXd& lx, Eigen::VectorXd& ly, Eigen::VectorXd& lz, Eigen::VectorXd& ux, Eigen::VectorXd& uy, Eigen::VectorXd& uz){
		// lower and upper bounds for equality and inequality
		lx.resize(this->constraintNum_);
		ux.resize(this->constraintNum_);
		ly.resize(this->constraintNum_);
		uy.resize(this->constraintNum_);
		lz.resize(this->constraintNum_);
		uz.resize(this->constraintNum_);
		int countConstraint = 0;
		
		// Position Bound: 2 + (k-1) + (k-1)
		{
			{   
				// ================2 Endpoint====================
				int pathStartIdx = 0; // start
				lx(countConstraint) = this->path_[pathStartIdx].x;
				ux(countConstraint) = this->path_[pathStartIdx].x;

				ly(countConstraint) = this->path_[pathStartIdx].y;
				uy(countConstraint) = this->path_[pathStartIdx].y;

				lz(countConstraint) = this->path_[pathStartIdx].z;
				uz(countConstraint) = this->path_[pathStartIdx].z;

				++countConstraint;

				int pathEndIdx = this->path_.size()-1;
				lx(countConstraint) = this->path_[pathEndIdx].x;
				ux(countConstraint) = this->path_[pathEndIdx].x;

				ly(countConstraint) = this->path_[pathEndIdx].y;
				uy(countConstraint) = this->path_[pathEndIdx].y;

				lz(countConstraint) = this->path_[pathEndIdx].z;
				uz(countConstraint) = this->path_[pathEndIdx].z;
				
				++countConstraint;
 			}

 			// ================k-1 Midpoint=====================
 			{
	 			for (int i=0; i<this->path_.size()-2; ++i){
	 				int pathIdx = i+1;
	 				// cout << this->path_[pathIdx] << endl;
	 				lx(countConstraint) = this->path_[pathIdx].x;
	 				ux(countConstraint) = this->path_[pathIdx].x;
	 				
	 				ly(countConstraint) = this->path_[pathIdx].y;
	 				uy(countConstraint) = this->path_[pathIdx].y;

	 				lz(countConstraint) = this->path_[pathIdx].z;
	 				uz(countConstraint) = this->path_[pathIdx].z;
	 				
	 				++countConstraint;
	 			}
 			}

 			// ================k-1 Continuity=====================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					int pathIdx = i+1;
					lx(countConstraint) = 0.0;
	 				ux(countConstraint) = 0.0;
	 				
	 				ly(countConstraint) = 0.0;
	 				uy(countConstraint) = 0.0;

	 				lz(countConstraint) = 0.0;
	 				uz(countConstraint) = 0.0;
	 				
	 				++countConstraint;
				}
			}
		}


		// Velocity Bound: 2 + (k-1)
		{	
			// ==================2 Endpoint=====================
			{
				// Start vel = 0
				lx(countConstraint) = 0.0;
	 			ux(countConstraint) = 0.0;
	 				
	 			ly(countConstraint) = 0.0;
	 			uy(countConstraint) = 0.0;

	 			lz(countConstraint) = 0.0;
	 			uz(countConstraint) = 0.0;

				++countConstraint;
				
				// End vel = 0
				lx(countConstraint) = 0.0;
	 			ux(countConstraint) = 0.0;
	 				
	 			ly(countConstraint) = 0.0;
	 			uy(countConstraint) = 0.0;

	 			lz(countConstraint) = 0.0;
	 			uz(countConstraint) = 0.0;		
			
	 			++countConstraint;
			}


			// ================K-1 Continuity===================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					int pathIdx = i+1;
					lx(countConstraint) = 0.0;
	 				ux(countConstraint) = 0.0;
	 				
	 				ly(countConstraint) = 0.0;
	 				uy(countConstraint) = 0.0;

	 				lz(countConstraint) = 0.0;
	 				uz(countConstraint) = 0.0;
	 				
	 				++countConstraint;
				}
			}
		}
		

		// Accel Bound: 2 + (k-1)
		{
			// ==================2 Endpoint=====================
			{
				// Start accel = 0
				lx(countConstraint) = 0.0;
	 			ux(countConstraint) = 0.0;
	 				
	 			ly(countConstraint) = 0.0;
	 			uy(countConstraint) = 0.0;

	 			lz(countConstraint) = 0.0;
	 			uz(countConstraint) = 0.0;

				++countConstraint;
				
				// End vel = 0
				lx(countConstraint) = 0.0;
	 			ux(countConstraint) = 0.0;
	 				
	 			ly(countConstraint) = 0.0;
	 			uy(countConstraint) = 0.0;

	 			lz(countConstraint) = 0.0;
	 			uz(countConstraint) = 0.0;		
			
	 			++countConstraint;
			}


			// ================K-1 Continuity===================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					int pathIdx = i+1;
					lx(countConstraint) = 0.0;
	 				ux(countConstraint) = 0.0;
	 				
	 				ly(countConstraint) = 0.0;
	 				uy(countConstraint) = 0.0;

	 				lz(countConstraint) = 0.0;
	 				uz(countConstraint) = 0.0;
	 				
	 				++countConstraint;
				}
			}
		}


		// Higher order
		{
			// Jerk Bound: k-1 
			if (this->diffDegree_ >= 3){
				// ================K-1 Continuity===================
				for (int i=0; i<this->path_.size()-2; ++i){
					int pathIdx = i+1;
					lx(countConstraint) = 0.0;
	 				ux(countConstraint) = 0.0;
	 				
	 				ly(countConstraint) = 0.0;
	 				uy(countConstraint) = 0.0;

	 				lz(countConstraint) = 0.0;
	 				uz(countConstraint) = 0.0;
	 				
	 				++countConstraint;
				}
			}
		

			// Snap Bound: k-1
			if (this->diffDegree_ >= 4){
				// ================K-1 Continuity===================
				for (int i=0; i<this->path_.size()-2; ++i){
					int pathIdx = i+1;
					lx(countConstraint) = 0.0;
	 				ux(countConstraint) = 0.0;
	 				
	 				ly(countConstraint) = 0.0;
	 				uy(countConstraint) = 0.0;

	 				lz(countConstraint) = 0.0;
	 				uz(countConstraint) = 0.0;
	 				
	 				++countConstraint;
				}
			}
		}
	}

	trajPlanner::pose polyTrajSolver::getPose(double t){
		trajPlanner::pose p;
		for (int i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				if (t == 0){
					t += 1;
				}
				int coeffStartIdx = (this->polyDegree_+1) * i;
				double x = 0; double y = 0; double z = 0; double yaw = 0;
				for (int d=0; d<this->polyDegree_+1; ++d){
					x += this->xSol_(coeffStartIdx+d) * pow(t, d);
					y += this->ySol_(coeffStartIdx+d) * pow(t, d);
					z += this->zSol_(coeffStartIdx+d) * pow(t, d);
				}

				double dx = 0; double dy = 0;
				for (int d=0; d<this->polyDegree_+1; ++d){
					dx += d * this->xSol_(coeffStartIdx+d) * pow(t, d-1);
					dy += d * this->ySol_(coeffStartIdx+d) * pow(t, d-1);
				}
				yaw = atan2(dy, dx);
				p.x = x; p.y = y; p.z = z; p.yaw = yaw;
				break;
			}
		}
		return p;
	}

	void polyTrajSolver::getTrajectory(std::vector<trajPlanner::pose>& trajectory, double delT){
		this->trajectory_.clear();
		double endTime = this->desiredTime_[this->desiredTime_.size()-1];
		for (double t=0; t<endTime; t+=delT){
			trajPlanner::pose p = this->getPose(t);
			trajectory.push_back(p);
			this->trajectory_.push_back(p);
		}
	}


	void polyTrajSolver::solve(){
		if (not this->init_){
			this->setUpProblem();
		}

		this->xWorker_ = std::thread(&polyTrajSolver::solveX, this);
		this->yWorker_ = std::thread(&polyTrajSolver::solveY, this);
		this->zWorker_ = std::thread(&polyTrajSolver::solveZ, this);

		this->xWorker_.join();
		this->yWorker_.join();
		this->zWorker_.join();

		// cout << "Solution: " << endl;
		// cout << "x: " << endl;
		// cout << this->xSol_ << endl;
		// cout << "y: " << endl;
		// cout << this->ySol_ << endl;
		// cout << "z: " << endl;
		// cout << this->zSol_ << endl;

	}

	void polyTrajSolver::solveX(){
		 if (this->xSolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
		 this->xSol_ = this->xSolver_->getSolution();
	}

	void polyTrajSolver::solveY(){
		 if (this->ySolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
		 this->ySol_ = this->ySolver_->getSolution();
	}

	void polyTrajSolver::solveZ(){
 		if (this->zSolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
 		this->zSol_ = this->zSolver_->getSolution();
	}

}