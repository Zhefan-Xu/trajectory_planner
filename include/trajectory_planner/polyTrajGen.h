#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_planner/utils.h>
#include <trajectory_planner/QuadProg++.hh>
#include <chrono> 
using namespace std::chrono;

namespace trajPlanner{
	class polyTraj{
	private:
		int degree;
		int diff_degree;
		double velocityd;
		std::vector<pose> path;
		std::vector<pose> trajectory;
		std::vector<double> timed; // desired time for path
		double perturb; // convert PSD -> PD


		// the following attributes are based on Quadprog++
		quadprogpp::Matrix<double> Qx, Qy, Qz; // Hessian
		quadprogpp::Vector<double> px, py, pz; // Linear term in objective function
		quadprogpp::Matrix<double> Ax, Ay, Az; // Linear Equality Constraint Matrix
		quadprogpp::Vector<double> bx, by, bz; // Linear Equality Constraint constant vector
		quadprogpp::Matrix<double> Cx, Cy, Cz; // Linear Inequality Constraint Matrix
		quadprogpp::Vector<double> dx, dy, dz; // LInear Inequality Constraint Vector
		quadprogpp::Vector<double> x_param_sol, y_param_sol, z_param_sol; // Solution



	public:
		polyTraj();
		polyTraj(int _degree);
		polyTraj(int _degree, double _velocityd, int _diff_degree);
		polyTraj(int _degree, double _velocityd, int _diff_degree, double _perturb);
		void adjustTimed(const std::vector<double>& _timed); // same format as timed
		void adjustTimedSegment(const std::vector<double>& time_segment); // time for each segment
		void adjustWaypoint(const std::vector<int> &collision_idx, double delT);
		void adjustCorridorConstraint(const std::set<int> &collision_seg, const std::vector<double> &radius, double delT);
		std::set<int> findCollisionSegment(const std::vector<int> &collision_seg, double delT);
		pose getPoseLineInterpolate(double seg_idx, double t);
		std::map<int, std::vector<double>> findCorridorConstraintTime(const std::set<int> &collision_seg, double delT); // find time to add corridor constraints
		std::vector<pose> getAddPath(const std::set<int>& collision_seg);
		void loadWaypointPath(const std::vector<pose> &_path);
		void constructQp(); // Hessian Matrix and linear vector
		void constructAb(); // Equality constraint
		void constructCd(); // Inequality constraint
		void constructCd(const std::set<int> &collision_seg, const std::vector<double> &radius, double delT);
		void optimize();
		pose getPose(double t);
		std::vector<pose> getTrajectory(double delT);
		std::vector<pose> getWaypointPath();
		std::vector<double> getTimed();
		void getSol(quadprogpp::Vector<double>& _x_param_sol, quadprogpp::Vector<double>& _y_param_sol,quadprogpp::Vector<double>& _z_param_sol);
		void printWaypointPath();
		void printTrajectory();
	};
}

#endif