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
		this->xSolver_.reset(new OsqpEigen::Solver ());
		this->ySolver_.reset(new OsqpEigen::Solver ());
		this->zSolver_.reset(new OsqpEigen::Solver ());

		// Soft constraint for waypoint (default: fasle)
		this->softConstraint_ = false;
		for (int i=0; i<3; ++i){
			this->scDeviation_[i] = 0; // soft constraint for x y z
		}

		// Corridor constraint (default: false)
		this->corridorConstraint_ = false;

		this->init_ = false;

		this->setDefaultInit(); // initial condition for vel and acc
	}


	polyTrajSolver::polyTrajSolver(int polyDegree, int diffDegree, int continuityDegree, double desiredVel, double desiredAcc)
	: polyDegree_(polyDegree), diffDegree_(diffDegree), continuityDegree_(continuityDegree), desiredVel_(desiredVel), desiredAcc_(desiredAcc){
		this->xSolver_.reset(new OsqpEigen::Solver ());
		this->ySolver_.reset(new OsqpEigen::Solver ());
		this->zSolver_.reset(new OsqpEigen::Solver ());

		// Soft constraint for waypoint (default: fasle)
		this->softConstraint_ = false;
		for (int i=0; i<3; ++i){
			this->scDeviation_[i] = 0; // soft constraint for x y z
		}

		// Corridor constraint (default: false)
		this->corridorConstraint_ = false;

		this->init_ = false;

		this->setDefaultInit(); // initial condition for vel and acc
	}
	

	void polyTrajSolver::updatePath(const std::vector<trajPlanner::pose>& path){
		this->path_ = path;
		int pathSegNum = this->path_.size()-1;
		this->paramDim_ = (this->polyDegree_+1) * pathSegNum;
		if (this->continuityDegree_ - 2 < 0){this->continuityDegree_ = 2;}
		this->constraintNum_ = this->getConstraintNum(); 
		this->avgTimeAllocation();
		// this->equalTimeAllocation();
		this->init_ = false;
	}

	void polyTrajSolver::updateInitVel(double vx, double vy, double vz){
		geometry_msgs::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateInitVel(v);
	}

	void polyTrajSolver::updateInitVel(const geometry_msgs::Twist& v){
		this->initVel_ = v;
	}


	void polyTrajSolver::updateEndVel(double vx, double vy, double vz){
		geometry_msgs::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateEndVel(v);
	}

	void polyTrajSolver::updateEndVel(const geometry_msgs::Twist& v){
		this->endVel_ = v;
	}

	void polyTrajSolver::updateInitAcc(double ax, double ay, double az){
		geometry_msgs::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateInitAcc(a);
	}


	void polyTrajSolver::updateInitAcc(const geometry_msgs::Twist& a){
		this->initAcc_ = a;
	}

	void polyTrajSolver::updateEndAcc(double ax, double ay, double az){
		geometry_msgs::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateEndAcc(a);
	}


	void polyTrajSolver::updateEndAcc(const geometry_msgs::Twist& a){
		this->endAcc_ = a;
	}
	
	void polyTrajSolver::setDefaultInit(){
		// initialize 
		this->updateInitVel(0, 0, 0);
		this->updateEndVel(0, 0, 0);
		this->updateInitAcc(0, 0, 0);
		this->updateEndAcc(0, 0, 0); 
	}


	void polyTrajSolver::avgTimeAllocation(){
		double totalTime = 0;
		this->desiredTime_.clear();
		this->desiredTime_.push_back(totalTime);
		double distance;
		for (size_t i=0; i<this->path_.size(); ++i){
			if (i != 0){
				distance = trajPlanner::getPoseDistance(this->path_[i], this->path_[i-1]);
				double duration = (double) distance/this->desiredVel_;
				totalTime += duration;
				this->desiredTime_.push_back(totalTime);
			}
		}
	}

	void polyTrajSolver::equalTimeAllocation(){
		this->desiredTime_.clear();
		double totalDistance = 0.0;
		for (size_t i=0; i<this->path_.size(); ++i){
			if (i != 0){
				double distance = trajPlanner::getPoseDistance(this->path_[i], this->path_[i-1]);
				totalDistance += distance;
			}
		}

		double totalTime = totalDistance / this->desiredVel_;
		for (size_t i=0; i<this->path_.size(); ++i){
			this->desiredTime_.push_back(i * totalTime/(this->path_.size()-1));
		}
	}

	int polyTrajSolver::getConstraintNum(){
		int pathSegNum = this->path_.size()-1;
		int num = (2+ pathSegNum-1 + pathSegNum-1) + (2+pathSegNum-1) + (2+pathSegNum-1) + (pathSegNum-1) * (this->continuityDegree_-2); // position, velocity, acceleration, jerk, snap
		return num;
	}

	void polyTrajSolver::setUpProblem(){
		// set three solvers' argument
		Eigen::SparseMatrix<double> P; // Hessian matrix
		this->constructP(P); 

		Eigen::VectorXd q;
		this->constructQ(q); // gradient vector (first order term)

		Eigen::SparseMatrix<double> A; // Equality and inquality matrix
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

		//debug
		// Eigen::MatrixXd pDebug;
		// pDebug.resize(this->paramDim_, this->paramDim_);
		// pDebug.setZero();


		// construct the coefficient matrix
		// each segment
		// cout << "diff degree: " << this->diffDegree_ << endl;
		for (size_t n=0; n<this->path_.size()-1; ++n){
			for (int i=this->diffDegree_; i<this->polyDegree_+1; ++i){
				for (int j=this->diffDegree_; j<this->polyDegree_+1; ++j){
					double factor = 1.0;
					for (int d=0; d<this->diffDegree_; ++d){
						factor *= (double) (i - d) * (j - d);
					}
					factor /= (double) (i + j - this->diffDegree_*2 + 1);
					// double startTime = 0.0;
					// double endTime = this->desiredTime_[n+1];
					// factor *= (double) pow(endTime, i+j-5) - pow(startTime, i+j-5);
					P.insert(n*(this->polyDegree_+1) + i, n*(this->polyDegree_+1) + j) = factor;
					// pDebug(n*(this->polyDegree_+1) + i, n*(this->polyDegree_+1) + j) = factor;
				}
			}
		}
		// cout << "p debug: " << endl;
		// cout << pDebug << endl;
		// // assign parameters for each path segment
		// for (int i=0; i<this->path_.size()-1; ++i){
		// 	// go through each degree order from 0 to polynomial degree
		// 	for (int d=0; d<this->polyDegree_+1; ++d){
		// 		if (d < this->diffDegree_){
		// 			coeff(d) = 0;
		// 		}
		// 		else{
		// 			double factor = 1.0;
		// 			for (int j=0; j<this->diffDegree_-1; ++j){
		// 				factor *= (d-j);
		// 			}
		// 			// each segment start from 0 to its duration
		// 			double startTime = 0.0;
		// 			double endTime = 1.0;
		// 			// factor *= pow(endTime, d-this->diffDegree_+1) - pow(startTime, d-this->diffDegree_+1);
		// 			coeff(d) = factor;
		// 		}
		// 	}
		// 	m = coeff * coeff.transpose();


		// 	int startIdx = (this->polyDegree_+1) * i;
		// 	for (int row=0; row<m.rows(); ++row){
		// 		for (int col=0; col<m.cols(); ++col){
		// 			double value = m(row, col);
		// 			if (value != 0){
		// 				P.insert(startIdx+row, startIdx+col) = value;
		// 			}
		// 		}
		// 	}
		// }
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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

			// ================1 Endpoint (start)===============================
			{
				// Note velocity = 0
				double startIdx = 0;
				double startTime = 0.0;
				for (int d=0; d<this->polyDegree_+1; ++d){ // start index
					if (d > 1){
						double factor = d * (d-1) * pow(startTime, d-2);
						if (factor != 0){
							A.insert(countConstraint, startIdx+d) = factor;
						}
					}
				}
				++countConstraint;

				int endIdx = (this->path_.size()-2) * (this->polyDegree_+1);
				double endTime = 1.0;
				for (int d=0; d<this->polyDegree_+1; ++d){ // start index
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-1; ++i){
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
		// // cout << "Expected constraints: " << this->constraintNum_ << endl;
		// cout << "A debug:" << endl;
		// cout << A << endl;
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
 					for (size_t i=0; i<this->path_.size()-2; ++i){
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
 					for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				lx(countConstraint) = this->initVel_.linear.x;
	 			ux(countConstraint) = this->initVel_.linear.x;
	 				
	 			ly(countConstraint) = this->initVel_.linear.y;
	 			uy(countConstraint) = this->initVel_.linear.y;

	 			lz(countConstraint) = this->initVel_.linear.z;
	 			uz(countConstraint) = this->initVel_.linear.z;

				++countConstraint;
				
				// // End vel = 0
				lx(countConstraint) = this->endVel_.linear.x;
	 			ux(countConstraint) = this->endVel_.linear.x;
	 				
	 			ly(countConstraint) = this->endVel_.linear.y;
	 			uy(countConstraint) = this->endVel_.linear.y;

	 			lz(countConstraint) = this->endVel_.linear.z;
	 			uz(countConstraint) = this->endVel_.linear.z;		
			
	 			++countConstraint;
			}


			// ================K-1 Continuity===================
			{
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				// Start acc = 0
				lx(countConstraint) = this->initAcc_.linear.x;
	 			ux(countConstraint) = this->initAcc_.linear.x;
	 				
	 			ly(countConstraint) = this->initAcc_.linear.y;
	 			uy(countConstraint) = this->initAcc_.linear.y;

	 			lz(countConstraint) = this->initAcc_.linear.z;
	 			uz(countConstraint) = this->initAcc_.linear.z;

				++countConstraint;
				
				lx(countConstraint) = this->endAcc_.linear.x;
	 			ux(countConstraint) = this->endAcc_.linear.x;
	 				
	 			ly(countConstraint) = this->endAcc_.linear.y;
	 			uy(countConstraint) = this->endAcc_.linear.y;

	 			lz(countConstraint) = this->endAcc_.linear.z;
	 			uz(countConstraint) = this->endAcc_.linear.z;

				++countConstraint;
			}

			// ================K-1 Continuity===================
			{
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-2; ++i){
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
				for (size_t i=0; i<this->path_.size()-1; ++i){
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

		// cout << "lx debug: " << endl;
		// cout << lx << endl;
		// cout << "ux debug: " << endl;
		// cout << ux << endl;
	}


	void polyTrajSolver::solve(){
		if (not this->init_){
			this->setUpProblem();
		}	
		else{
			this->updateProblem();
		}

		// this->xWorker_ = std::thread(&polyTrajSolver::solveX, this);
		// this->yWorker_ = std::thread(&polyTrajSolver::solveY, this);
		// this->zWorker_ = std::thread(&polyTrajSolver::solveZ, this);

		// this->xWorker_.join();
		// this->yWorker_.join();
		// this->zWorker_.join();
		this->solveX();
		this->solveY();
		this->solveZ();

	}

	void polyTrajSolver::solveX(){
		if (this->xSolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
		this->xSol_ = this->xSolver_->getSolution();
		// cout << "x sol: " << this->xSol_ << endl;
		for (size_t n=0; n<this->path_.size()-1; ++n){
			for (int d=0; d<=this->polyDegree_; ++d){
			 	this->xSol_(n*(this->polyDegree_+1) + d) /= pow((this->desiredTime_[n+1] - this->desiredTime_[n]), d);
			}
		}
		// cout << "x sol: " << this->xSol_ << endl;
	}

	void polyTrajSolver::solveY(){
		if (this->ySolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
		this->ySol_ = this->ySolver_->getSolution();
		// cout << "y sol: " << this->ySol_ << endl;
		for (size_t n=0; n<this->path_.size()-1; ++n){
			for (int d=0; d<=this->polyDegree_; ++d){
			 	this->ySol_(n*(this->polyDegree_+1) + d) /= pow((this->desiredTime_[n+1] - this->desiredTime_[n]), d);
			}
		}
		 // cout << "y sol: " << this->ySol_ << endl;
	}

	void polyTrajSolver::solveZ(){
 		if (this->zSolver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;
 		this->zSol_ = this->zSolver_->getSolution();
 		// cout << "z sol: " << this->zSol_ << endl;
		for (size_t n=0; n<this->path_.size()-1; ++n){
			for (int d=0; d<=this->polyDegree_; ++d){
			 	this->zSol_(n*(this->polyDegree_+1) + d) /= pow((this->desiredTime_[n+1] - this->desiredTime_[n]), d);
			}
		}
		 // cout << "z sol: " << this->zSol_ << endl;
	}


	void polyTrajSolver::evalTrajectory(){
		double delT = 0.1;
		std::vector<trajPlanner::pose> trajectory; 
		this->getTrajectory(trajectory, delT);


		std::vector<double> trajLengthVec;
		int timeIdx = 1;
		double trajLength = 0.0;
		// evaluate each segment length of trajectory
		for (double i=0; i<trajectory.size()-1; ++i){
			double t = i * delT;
			trajPlanner::pose p1 = trajectory[i];
			trajPlanner::pose p2 = trajectory[i+1];
			if ((t <= this->desiredTime_[timeIdx]) and (i != trajectory.size() - 2)){
				trajLength += trajPlanner::getPoseDistance(p1, p2);
			}
			else{
				// cout << "traj " << timeIdx << ": " << trajLength << endl;
				trajLengthVec.push_back(trajLength);
				trajLength = 0.0;
				timeIdx += 1;
			}
		}

		// evaluate each segment legnth of raw path
		std::vector<double> pathLegnthVec;
		for (size_t i=0; i<this->path_.size()-1; ++i){
			trajPlanner::pose p1 = this->path_[i];
			trajPlanner::pose p2 = this->path_[i+1];
			double pathLength = trajPlanner::getPoseDistance(p1, p2);
			pathLegnthVec.push_back(pathLength);
			// cout << "path " << i << ": " << pathLength << endl;
		}
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
		for (size_t i=0; i<this->path_.size()-1; ++i){
			corridorSizeVec.push_back(corridorSize);
		}
		this->setCorridorConstraint(corridorSizeVec, corridorRes);
	}

	void polyTrajSolver::updateCorridorParam(){
		// update number of constraint, determine which time each segment needs to consider
		this->segToTimePose_.clear();
		int countCorridorConstraint = 0;
		for (size_t i=0; i<this->path_.size()-1; ++i){ // go through each path segment
			if (this->corridorSizeVec_[i] == 0.0){ // this segment does not need corridor constraint
				std::unordered_map<double, trajPlanner::pose> timeToPose;
				this->segToTimePose_.push_back(timeToPose);// dummy for index matching
				continue;
			}
			trajPlanner::pose pStart = this->path_[i];
			trajPlanner::pose pEnd = this->path_[i+1]; 
			double duration = this->desiredTime_[i+1] - this->desiredTime_[i];
			int numCorridor = ceil(duration * this->corridorRes_);
			// int numCorridor = this->corridorRes_;
			double dt = (1.0)/numCorridor;
			std::unordered_map<double, trajPlanner::pose> timeToPose;
			for (double t=0; t<=1.0; t+=dt){
				trajPlanner::pose pMid = this->interpolatePose(pStart, pEnd, 0.0, 1.0, t);
				timeToPose[t] = pMid;
				++countCorridorConstraint;
			}

			this->segToTimePose_.push_back(timeToPose);
		}
		if (this->continuityDegree_ - 2 < 0){this->continuityDegree_ = 2;}
		this->constraintNum_ = this->getConstraintNum() + countCorridorConstraint;
	}

	trajPlanner::pose polyTrajSolver::interpolatePose(const trajPlanner::pose& pStart, const trajPlanner::pose& pEnd, double startTime, double endTime, double t){
		trajPlanner::pose pMid;
		double xMid = pStart.x + (pEnd.x - pStart.x) * (t-startTime)/(endTime-startTime);
		double yMid = pStart.y + (pEnd.y - pStart.y) * (t-startTime)/(endTime-startTime);
		double zMid = pStart.z + (pEnd.z - pStart.z) * (t-startTime)/(endTime-startTime);
		pMid.x = xMid;
		pMid.y = yMid;
		pMid.z = zMid;
		return pMid;
	}


	trajPlanner::pose polyTrajSolver::getPose(double t){
		trajPlanner::pose p;
		for (size_t i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				t = (double) (t-startTime);///(endTime - startTime);
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

	Eigen::Vector3d polyTrajSolver::getPos(double t){
		Eigen::Vector3d pos;
		for (size_t i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				t = (double) (t-startTime);///(endTime - startTime);
				int coeffStartIdx = (this->polyDegree_+1) * i;
				double x = 0; double y = 0; double z = 0; 
				for (int d=0; d<this->polyDegree_+1; ++d){
					x += this->xSol_(coeffStartIdx+d) * pow(t, d);
					y += this->ySol_(coeffStartIdx+d) * pow(t, d);
					z += this->zSol_(coeffStartIdx+d) * pow(t, d);
				}

				pos(0) = x; pos(1) = y; pos(2) = z;
				break;
			}
		}
		return pos;		
	}

	Eigen::Vector3d polyTrajSolver::getVel(double t){
		Eigen::Vector3d vel;
		for (size_t i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				t = (double) (t-startTime);///(endTime - startTime);
				int coeffStartIdx = (this->polyDegree_+1) * i;
				double vx = 0; double vy = 0; double vz = 0; 
				for (int d=1; d<this->polyDegree_+1; ++d){
					vx += this->xSol_(coeffStartIdx+d) * d * pow(t, d-1);
					vy += this->ySol_(coeffStartIdx+d) * d * pow(t, d-1);
					vz += this->zSol_(coeffStartIdx+d) * d * pow(t, d-1);
				}

				vel(0) = vx; vel(1) = vy; vel(2) = vz;
				break;
			}
		}
		return vel;
	}

	Eigen::Vector3d polyTrajSolver::getAcc(double t){
		Eigen::Vector3d acc;
		for (size_t i=0; i<this->desiredTime_.size()-1; ++i){
			double startTime = this->desiredTime_[i];
			double endTime = this->desiredTime_[i+1];
			if ((t >= startTime) and (t <= endTime)){
				t = (double) (t-startTime);///(endTime - startTime);
				int coeffStartIdx = (this->polyDegree_+1) * i;
				double ax = 0; double ay = 0; double az = 0;
				for (int d=2; d<this->polyDegree_+1; ++d){
					ax += this->xSol_(coeffStartIdx+d) * d * (d-1) * pow(t, d-1);
					ay += this->ySol_(coeffStartIdx+d) * d * (d-1) * pow(t, d-2);
					az += this->zSol_(coeffStartIdx+d) * d * (d-1) * pow(t, d-2);
				}

				acc(0) = ax; acc(1) = ay; acc(2) = az;
				break;
			}
		}
		return acc;
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