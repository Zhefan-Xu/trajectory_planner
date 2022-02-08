/*
	File: polyTrajSolver.cpp
	------------------------
	function definition of polynomial trajectory solver

*/ 
#include <trajectory_planner/polyTrajSolver.h>

namespace trajPlanner{
	polyTrajSolver::polyTrajSolver(){}

	polyTrajSolver::polyTrajSolver(int polyDegree, int diffDegree, int continuityDegree, double desiredVel)
	: polyDegree_(polyDegree), diffDegree_(diffDegree), continuityDegree_(continuityDegree), desiredVel_(desiredVel){
		this->xSolver_ = new OsqpEigen::Solver ();
		this->ySolver_ = new OsqpEigen::Solver ();
		this->zSolver_ = new OsqpEigen::Solver ();
		// this->xSolver_->settings()->setMaxIteration(10000);
		// this->ySolver_->settings()->setMaxIteration(10000);
		// this->zSolver_->settings()->setMaxIteration(10000);

		// Soft constraint for waypoint (default: fasle)
		this->softConstraint_ = false;
		for (int i=0; i<3; ++i){
			this->scDeviation_[i] = 0; // soft constraint for x y z
		}

		// Corridor constraint (default: false)
		this->corridorConstraint_ = false;

		this->init_ = false;
	}
	

	void polyTrajSolver::updatePath(const std::vector<trajPlanner::pose>& path){
		this->path_ = path;
		int pathSegNum = this->path_.size()-1;
		this->paramDim_ = (this->polyDegree_+1) * pathSegNum;
		if (this->continuityDegree_ - 2 < 0){this->continuityDegree_ = 2;}
		this->constraintNum_ = (2+ pathSegNum-1 + pathSegNum-1) + (2+pathSegNum-1) + (pathSegNum-1) + (pathSegNum-1) * (this->continuityDegree_-2); // position, velocity, acceleration, jerk, snap
		this->avgTimeAllocation();
	}

	void polyTrajSolver::avgTimeAllocation(){
		double totalTime = 0;
		this->desiredTime_.clear();
		this->desiredTime_.push_back(totalTime);
		double distance;
		for (int i=0; i<this->path_.size(); ++i){
			if (i != 0){
				distance = trajPlanner::getPoseDistance(this->path_[i], this->path_[i-1]);
				double duration = (double) distance/this->desiredVel_;
				totalTime += duration;
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
		this->xSolver_->clearSolver();
		this->xSolver_->data()->clearHessianMatrix();
		this->xSolver_->data()->clearLinearConstraintsMatrix();
		this->xSolver_->settings()->setVerbosity(false);
		this->xSolver_->data()->setNumberOfVariables(this->paramDim_);
		this->xSolver_->data()->setNumberOfConstraints(this->constraintNum_); // constraint number will be updated when using point interpolation
		if(!this->xSolver_->data()->setHessianMatrix(P)) return;
    	if(!this->xSolver_->data()->setGradient(q)) return;
    	if(!this->xSolver_->data()->setLinearConstraintsMatrix(A)) return;
    	if(!this->xSolver_->data()->setLowerBound(lx)) return;
    	if(!this->xSolver_->data()->setUpperBound(ux)) return;
    	if(!this->xSolver_->initSolver()) return;

    	// y solver:
    	this->ySolver_->clearSolver();
    	this->ySolver_->data()->clearHessianMatrix();
		this->ySolver_->data()->clearLinearConstraintsMatrix();
    	this->ySolver_->settings()->setVerbosity(false);
    	this->ySolver_->data()->setNumberOfVariables(this->paramDim_);
    	this->ySolver_->data()->setNumberOfConstraints(this->constraintNum_);
		if(!this->ySolver_->data()->setHessianMatrix(P)) return;
    	if(!this->ySolver_->data()->setGradient(q)) return;
    	if(!this->ySolver_->data()->setLinearConstraintsMatrix(A)) return;
    	if(!this->ySolver_->data()->setLowerBound(ly)) return;
    	if(!this->ySolver_->data()->setUpperBound(uy)) return;
    	if(!this->ySolver_->initSolver()) return;
    	

    	// z solver:
    	this->zSolver_->clearSolver();
    	this->zSolver_->data()->clearHessianMatrix();
		this->zSolver_->data()->clearLinearConstraintsMatrix();
    	this->zSolver_->settings()->setVerbosity(false);
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

	void polyTrajSolver::updateProblem(){
		Eigen::VectorXd lx, ly, lz; // lower bound
		Eigen::VectorXd ux, uy, uz; // uppper bound
		this->constructBound(lx, ly, lz, ux, uy, uz);

		// x solver
		this->xSolver_->updateBounds(lx, ux);

		// y solver
		this->ySolver_->updateBounds(ly, uy);

		// z solver
		this->zSolver_->updateBounds(lz, uz);
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
					// each segment start from 0 to its duration
					double startTime = 0.0;
					double endTime = 1.0;
					factor *= pow(endTime, d-this->diffDegree_+1) - pow(startTime, d-this->diffDegree_+1);
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
				double startTime = 0.0;
				for (int d=0; d<this->polyDegree_+1; ++d){ // start position
				 	double factor = pow(startTime, d);
				 	if (factor != 0){
				 		A.insert(countConstraint, startIdx+d) = factor;
					}
				}
				++countConstraint;

				int endIdx = (this->path_.size()-2) * (this->polyDegree_+1);
				double endTime = 1.0;
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
					double currTime = 1.0;
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
					double leftTime = 1.0; // time for "left" piece of polynomial
					double rightTime = 0.0; // time for "right" piece of polynomial
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						double leftFactor = pow(leftTime, d);
						double rightFactor = pow(rightTime, d);
						if (leftFactor != 0){
							A.insert(countConstraint, leftStartIdx+d) = leftFactor;
						}
						if (rightFactor != 0){
							A.insert(countConstraint, rightStartIdx+d) = -rightFactor;
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
				double startTime = 0.0;
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
				double endTime = 1.0;
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
					double leftTime = 1.0;
					double rightTime = 0.0;
					double dtLeft = this->desiredTime_[i+1] - this->desiredTime_[i];
					double dtRight = this->desiredTime_[i+2] - this->desiredTime_[i+1];
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d != 0){ // C0 index is alawys zero
							double leftFactor = d * pow(leftTime, d-1);
							double rightFactor = d * pow(rightTime, d-1);
							if (leftFactor != 0){
								A.insert(countConstraint, leftStartIdx+d) = leftFactor * dtRight;
							}
							if (rightFactor != 0){
								A.insert(countConstraint, rightStartIdx+d) = -rightFactor * dtLeft;
							}
						}
					}
					++countConstraint;
				}
			}
		}

		// Acceleration Constraint:
		{
			// ===============K-1 Continuity============================
			{
				for (int i=0; i<this->path_.size()-2; ++i){
					double leftTime = 1.0;
					double rightTime = 0.0;
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					double dtLeft = this->desiredTime_[i+1] - this->desiredTime_[i];
					double dtRight = this->desiredTime_[i+2] - this->desiredTime_[i+1];
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d > 1){
							double leftFactor = d * (d-1) * pow(leftTime, d-2);
							double rightFactor = d * (d-1) * pow(rightTime, d-2);
							if (leftFactor != 0){
								A.insert(countConstraint, leftStartIdx+d) = leftFactor * pow(dtRight,2);		
							}
							if (rightFactor != 0){
								A.insert(countConstraint, rightStartIdx+d) = -rightFactor * pow(dtLeft, 2);		
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
			if (this->continuityDegree_ >= 3){
				for (int i=0; i<this->path_.size()-2; ++i){
					double leftTime = 1.0;
					double rightTime = 0.0;
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					double dtLeft = this->desiredTime_[i+1] - this->desiredTime_[i];
					double dtRight = this->desiredTime_[i+2] - this->desiredTime_[i+1];
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d > 2){
							double leftFactor = d * (d-1) * (d-2) * pow(leftTime, d-3);
							double rightFactor = d * (d-1) * (d-2) * pow(rightTime, d-3);
							if (leftFactor != 0){
								A.insert(countConstraint, leftStartIdx+d) = leftFactor * pow(dtRight, 3);
							}
							if (rightFactor != 0){
								A.insert(countConstraint, rightStartIdx+d) = -rightFactor * pow(dtLeft, 3);
							}
						}
					}
					++countConstraint;
				}
			}

			//=================K-1 Conitnuity in Snap===================
			if (this->continuityDegree_ >= 4){
				for (int i=0; i<this->path_.size()-2; ++i){
					double leftTime = 1.0;
					double rightTime = 0.0;
					int leftStartIdx = (this->polyDegree_+1) * i;
					int rightStartIdx = (this->polyDegree_+1) * (i+1);
					double dtLeft = this->desiredTime_[i+1] - this->desiredTime_[i];
					double dtRight = this->desiredTime_[i+2] - this->desiredTime_[i+1];
					for (int d=0; d<this->polyDegree_+1; ++d){
						if (d>3){
							double leftFactor = d * (d-1) * (d-2) * (d-3) * pow(leftTime, d-4);
							double rightFactor = d * (d-1) * (d-2) * (d-3) * pow(rightTime, d-4);
							if (leftFactor != 0){
								A.insert(countConstraint, leftStartIdx+d) = leftFactor * pow(dtRight, 4);
							}
							if (rightFactor != 0){
								A.insert(countConstraint, rightStartIdx+d) = -rightFactor * pow(dtLeft, 4);
							}
							
						}
					}
					++countConstraint;
				}
			}
		}

		// Corridor Constraints:
		{
			if (this->corridorConstraint_){
				// go through each path segment and apply constraints
				for (int i=0; i<this->path_.size()-1; ++i){
					if (this->corridorSizeVec_[i] == 0.0){
						continue;
					}
					std::unordered_map<double, trajPlanner::pose> timeToPose = this->segToTimePose_[i];
					int coeffStartIdx = (this->polyDegree_+1) * i;
					for (auto itr : timeToPose){
						double t = itr.first;
						for (int d=0; d<this->polyDegree_+1; ++d){
							double factor = pow(t, d);
							if (factor != 0){
								A.insert(countConstraint, coeffStartIdx+d) = factor;	
							}
						}
						++countConstraint;
					}
				}
			}
		}
		// cout << "Number of constraints: " << countConstraint << endl;
		// cout << "Expected constraints: " << this->constraintNum_ << endl;
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
 				// Hard constraints:
 				if (not this->softConstraint_){
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
 				else{
 					for (int i=0; i<this->path_.size()-2; ++i){
		 				int pathIdx = i+1;
		 				// cout << this->path_[pathIdx] << endl;
		 				lx(countConstraint) = this->path_[pathIdx].x - this->scDeviation_[0];
		 				ux(countConstraint) = this->path_[pathIdx].x + this->scDeviation_[0];
		 				
		 				ly(countConstraint) = this->path_[pathIdx].y - this->scDeviation_[1];
		 				uy(countConstraint) = this->path_[pathIdx].y + this->scDeviation_[1];

		 				lz(countConstraint) = this->path_[pathIdx].z - this->scDeviation_[2];
		 				uz(countConstraint) = this->path_[pathIdx].z + this->scDeviation_[2];
		 				
		 				++countConstraint;
	 				}
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
		

		// Accel Bound: k-1
		{
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
			if (this->continuityDegree_ >= 3){
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
			if (this->continuityDegree_ >= 4){
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

		// Corridor Constraints:
		{
			if (this->corridorConstraint_){
				// go through each path segment and apply constraints
				for (int i=0; i<this->path_.size()-1; ++i){
					if (this->corridorSizeVec_[i] == 0.0){
						continue;
					}
					std::unordered_map<double, trajPlanner::pose> timeToPose = this->segToTimePose_[i];
					double r = this->corridorSizeVec_[i];
					for (auto itr : timeToPose){
						trajPlanner::pose p = itr.second;
						lx(countConstraint) = p.x - r;
		 				ux(countConstraint) = p.x + r;
		 				
		 				ly(countConstraint) = p.y - r;
		 				uy(countConstraint) = p.y + r;

		 				lz(countConstraint) = p.z - r;
		 				uz(countConstraint) = p.z + r;

						++countConstraint;
					}
				}
			}
		}
	}


	void polyTrajSolver::solve(){
		if (not this->init_){
			this->setUpProblem();
		}	
		else{
			this->updateProblem();
		}

		this->xWorker_ = std::thread(&polyTrajSolver::solveX, this);
		this->yWorker_ = std::thread(&polyTrajSolver::solveY, this);
		this->zWorker_ = std::thread(&polyTrajSolver::solveZ, this);

		this->xWorker_.join();
		this->yWorker_.join();
		this->zWorker_.join();
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

	void polyTrajSolver::setSoftConstraint(double r){
		this->softConstraint_ = true;
		for (int i=0; i<3; ++i){
			this->scDeviation_[i] = r;
		}
	}

	void polyTrajSolver::setSoftConstraint(double rx, double ry){
		this->softConstraint_ = true;
		this->scDeviation_[0] = rx; this->scDeviation_[1] = ry; this->scDeviation_[2] = 0.0;
	}

	void polyTrajSolver::setSoftConstraint(double rx, double ry, double rz){
		this->softConstraint_ = true;
		this->scDeviation_[0] = rx; this->scDeviation_[1] = ry; this->scDeviation_[2] = rz;	
	}


	void polyTrajSolver::setCorridorConstraint(const std::vector<double>& corridorSizeVec, double corridorRes){
		if (this->path_.size() <= 0){
			cout << "[Trajectory Solver]: Invalid! Please load path first!!" << endl;
			return;
		}
		this->corridorConstraint_ = true;
		this->corridorSizeVec_ = corridorSizeVec;
		this->corridorRes_ = corridorRes;
		this->updateCorridorParam(); 
	}	

	void polyTrajSolver::setCorridorConstraint(double corridorSize, double corridorRes){
		if (this->path_.size() <= 0){
			cout << "[Trajectory Solver]: Invalid! Please load path first!!" << endl;
			return;
		}
		std::vector<double> corridorSizeVec;
		for (int i=0; i<this->path_.size()-1; ++i){
			corridorSizeVec.push_back(corridorSize);
		}
		this->setCorridorConstraint(corridorSizeVec, corridorRes);
	}

	void polyTrajSolver::updateCorridorParam(){
		// update number of constraint, determine which time each segment needs to consider
		this->segToTimePose_.clear();
		int countCorridorConstraint = 0;
		for (int i=0; i<this->path_.size()-1; ++i){ // go through each path segment
			if (this->corridorSizeVec_[i] == 0.0){ // this segment does not need corridor constraint
				std::unordered_map<double, trajPlanner::pose> timeToPose;
				this->segToTimePose_.push_back(timeToPose);// dummy for index matching
				continue;
			}
			trajPlanner::pose pStart = this->path_[i];
			trajPlanner::pose pEnd = this->path_[i+1]; 
			double duration = this->desiredTime_[i+1] - this->desiredTime_[i];
			int numCorridor = (int) duration * this->corridorRes_;
			double dt = (1.0)/numCorridor;
			std::unordered_map<double, trajPlanner::pose> timeToPose;
			for (double t=0; t<=1.0; t+=dt){
				trajPlanner::pose pMid = this->interpolatePose(pStart, pEnd, 0.0, 1.0, t);
				timeToPose[t] = pMid;
				++countCorridorConstraint;
			}

			this->segToTimePose_.push_back(timeToPose);
		}
		int pathSegNum = this->path_.size()-1;
		if (this->continuityDegree_ - 2 < 0){this->continuityDegree_ = 2;}
		this->constraintNum_ = (2+ pathSegNum-1 + pathSegNum-1) + (2+pathSegNum-1) + (pathSegNum-1) + (pathSegNum-1) * (this->continuityDegree_-2) + countCorridorConstraint;; // position, velocity, acceleration, jerk, snap
	}

	trajPlanner::pose& polyTrajSolver::interpolatePose(const trajPlanner::pose& pStart, const trajPlanner::pose& pEnd, double startTime, double endTime, double t){
		static trajPlanner::pose pMid;
		double xMid = pStart.x + (pEnd.x - pStart.x) * (t-startTime)/(endTime-startTime);
		double yMid = pStart.y + (pEnd.y - pStart.y) * (t-startTime)/(endTime-startTime);
		double zMid = pStart.z + (pEnd.z - pStart.z) * (t-startTime)/(endTime-startTime);
		pMid.x = xMid;
		pMid.y = yMid;
		pMid.z = zMid;
		return pMid;
	}


	trajPlanner::pose& polyTrajSolver::getPose(double t){
		static trajPlanner::pose p;
		for (int i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				t = (double) (t-startTime)/(endTime - startTime);
				int coeffStartIdx = (this->polyDegree_+1) * i;
				double x = 0; double y = 0; double z = 0; double yaw = 0;
				for (int d=0; d<this->polyDegree_+1; ++d){
					x += this->xSol_(coeffStartIdx+d) * pow(t, d);
					y += this->ySol_(coeffStartIdx+d) * pow(t, d);
					z += this->zSol_(coeffStartIdx+d) * pow(t, d);
				}

				if (t==0){
					t = 0.01;
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
		trajectory.clear();
		this->trajectory_.clear();
		double endTime = this->desiredTime_[this->desiredTime_.size()-1];
		for (double t=0; t<endTime; t+=delT){
			trajPlanner::pose p = this->getPose(t);
			trajectory.push_back(p);
			this->trajectory_.push_back(p);
		}
		trajPlanner::pose endpoint = this->path_[this->path_.size()-1];
		trajectory.push_back(endpoint);
		this->trajectory_.push_back(endpoint);
	}

	std::vector<double>& polyTrajSolver::getTimeKnot(){
		return this->desiredTime_;
	}	

	void polyTrajSolver::getCorridor(std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, std::vector<double>& corridorSizeVec){
		segToTimePose = this->segToTimePose_;
		corridorSizeVec = this->corridorSizeVec_;
	}
}