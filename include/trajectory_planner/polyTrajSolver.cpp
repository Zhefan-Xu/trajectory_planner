/*
	File: polyTrajSolver.cpp
	------------------------
	function definition of polynomial trajectory solver

*/ 
#include <trajectory_planner/polyTrajSolver.h>

namespace trajPlanner{
	polyTrajSolver::polyTrajSolver(){}

	polyTrajSolver::polyTrajSolver(int polyDegree, int diffDegree)
	: polyDegree_(polyDegree), diffDegree_(diffDegree){
		// TODO:
		this->xSolver_ = new OsqpEigen::Solver ();
		this->ySolver_ = new OsqpEigen::Solver ();
		this->zSolver_ = new OsqpEigen::Solver ();
		this->init_ = false;
	}
	

	void polyTrajSolver::updatePath(const std::vector<trajPlanner::pose>& path){
		this->path_ = path;
	}

	void polyTrajSolver::setUpProblem(){

	}

	void polyTrajSolver::solve(){
		if (not this->init_){
			this->setUpProblem();
		}
	}
}