/*
 *    file:   nmpc_solver_setup.cpp
 *    author: Oskar Ljungqvist
 *    date:   2017-12-21
 *
 *    Comment: modified version of the nmpc_solver_setup.m works directly in ubutu.
 */

#include <acado_code_generation.hpp>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
// #include <acado_gnuplot.hpp>
#include <matrix_vector/vector.hpp>
#include <stdio.h>

USING_NAMESPACE_ACADO

int main( )
{

    double Ts = 0.1;  // prediction sampling time
    double N  = 30;   // Prediction horizon
    // // double g = 9.8066;
    // // double PI = 3.1415926535897932;
    DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState dummy1;
	// DifferentialState dummy1;
	// DifferentialState ax;
	// DifferentialState ay;
	// DifferentialState az;
	// // double yaw = currentYaw;

	// //Control Input
	Control ax;
	Control ay;
	Control az;
	Control sk_d;
	// Control sk_s;
	// // Control jx;
	// // Control jy;
	// // Control jz;

	// // MODEL Definition
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax; 
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(dummy1) == sk_d;
	// f << dot(dummy2) == sk_s;
	// // f << dot(ax) == jx;
	// // f << dot(ay) == jy;
	// // f << dot(az) == jz;

	// // Least Square Function
	Function h, hN;
	h << x << y << z << ax << ay << az << sk_d ;
	// << sk_d << sk_s;
	// h << jx;
	// h << jy;
	// h << jz;
	hN << x << y << z;
	// hN << 0;
	// hN << x << y << z << dummy1;



	// // r = this->getReferenceGoal();
    // // OnlineData r;

	// // setup OCP
	OCP ocp(0.0, N*Ts, N);
	DMatrix Q(7, 7);
	Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; 
	Q(6,6) = 1000.0;
	// Q(7,7) = 1000.0;
	
	DMatrix QN(3,3);
	QN.setIdentity(); QN(0,0) = 10.0; QN(1,1) = 10.0; QN(2,2) = 10.0;
	// DMatrix QN(1,1);
	// QN.setIdentity(); QN(0,0) = 1;
	

	// DMatrix QN(4,4);
	// QN.setIdentity(); QN(0,0) = 10.0; QN(1,1) = 10.0; QN(2,2) = 10.0;QN(3,3) = 1000.0;
	
	
	// // ocp.minimizeLSQ(Q, h, r); // Objective
	ocp.minimizeLSQ(Q, h); // Objective
	ocp.minimizeLSQEndTerm(QN, hN);
	// // Dynamic Constraint
	ocp.subjectTo(f); 


	// State constraint
	double slackRatio = 0.4;
	double skLimit = 1 - pow((1 - slackRatio), 2);
	// OnlineData vx_max;
    // OnlineData vy_max;
    // OnlineData vz_max;
    // OnlineData ax_max;
    // OnlineData ay_max;
    // OnlineData az_max;    
	
	// OnlineData obx1;
    // OnlineData oby1;
    // OnlineData obz1;
    // OnlineData a1;
    // OnlineData b1;
    // OnlineData c1;

	// ocp.setNOD(6);

	ocp.subjectTo( 1.0 <= z <= 1.5);
	ocp.subjectTo( -1 <= vx <= 1 );
	ocp.subjectTo( -1 <= vy <= 1 );
	ocp.subjectTo( -1 <= vz <= 1 );
	ocp.subjectTo( -1 <= ax <= 1 );
	ocp.subjectTo( -1 <= ay <= 1 );
	ocp.subjectTo( -1 <= az <= 1 );
	ocp.subjectTo(0 <= sk_d<= skLimit);
	// // ocp.subjectTo( -vx_max <= vx <= vx_max );
	// // ocp.subjectTo( -vy_max <= vy <= vy_max );
	// // ocp.subjectTo( -vz_max <= vz <= vz_max );
	// // ocp.subjectTo( -ax_max <= ax <= ax_max );
	// // ocp.subjectTo( -ay_max <= ay <= ay_max );
	// // ocp.subjectTo( -az_max <= az <= az_max );
	
	// ocp.subjectTo(0 <= sk_s <= skLimit);
	// // // // Control input constraint
	// // // ocp.subjectTo( -this->jx_max <= jx <= this->jx_max );
	// // // ocp.subjectTo( -this->jy_max <= jy <= this->jy_max );
	// // // ocp.subjectTo( -this->jz_max <= jz <= this->jz_max );

	OnlineData obx1;
    OnlineData oby1;
    OnlineData obz1;
    OnlineData a1;
    OnlineData b1;
    OnlineData c1;
	OnlineData yaw1;

	OnlineData obx2;
    OnlineData oby2;
    OnlineData obz2;
    OnlineData a2;
    OnlineData b2;
    OnlineData c2;
	OnlineData yaw2;

	OnlineData obx3;
    OnlineData oby3;
    OnlineData obz3;
    OnlineData a3;
    OnlineData b3;
    OnlineData c3;
	OnlineData yaw3;

	OnlineData obx4;
    OnlineData oby4;
    OnlineData obz4;
    OnlineData a4;
    OnlineData b4;
    OnlineData c4;
	OnlineData yaw4;

	OnlineData obx5;
    OnlineData oby5;
    OnlineData obz5;
    OnlineData a5;
    OnlineData b5;
    OnlineData c5;
	OnlineData yaw5;

	OnlineData obx6;
    OnlineData oby6;
    OnlineData obz6;
    OnlineData a6;
    OnlineData b6;
    OnlineData c6;
	OnlineData yaw6;

	OnlineData obx7;
    OnlineData oby7;
    OnlineData obz7;
    OnlineData a7;
    OnlineData b7;
    OnlineData c7;
	OnlineData yaw7;

	OnlineData obx8;
    OnlineData oby8;
    OnlineData obz8;
    OnlineData a8;
    OnlineData b8;
    OnlineData c8;
	OnlineData yaw8;

	OnlineData obx9;
    OnlineData oby9;
    OnlineData obz9;
    OnlineData a9;
    OnlineData b9;
    OnlineData c9;
	OnlineData yaw9;

	OnlineData obx10;
    OnlineData oby10;
    OnlineData obz10;
    OnlineData a10;
    OnlineData b10;
    OnlineData c10;
	OnlineData yaw10;

	OnlineData obx11;
    OnlineData oby11;
    OnlineData obz11;
    OnlineData a11;
    OnlineData b11;
    OnlineData c11;
	OnlineData yaw11;

	OnlineData obx12;
    OnlineData oby12;
    OnlineData obz12;
    OnlineData a12;
    OnlineData b12;
    OnlineData c12;
	OnlineData yaw12;

	OnlineData obx13;
    OnlineData oby13;
    OnlineData obz13;
    OnlineData a13;
    OnlineData b13;
    OnlineData c13;
	OnlineData yaw13;

	OnlineData obx14;
    OnlineData oby14;
    OnlineData obz14;
    OnlineData a14;
    OnlineData b14;
    OnlineData c14;
	OnlineData yaw14;

	OnlineData obx15;
    OnlineData oby15;
    OnlineData obz15;
    OnlineData a15;
    OnlineData b15;
    OnlineData c15;
	OnlineData yaw15;

	OnlineData obx16;
    OnlineData oby16;
    OnlineData obz16;
    OnlineData a16;
    OnlineData b16;
    OnlineData c16;
	OnlineData yaw16;

	OnlineData obx17;
    OnlineData oby17;
    OnlineData obz17;
    OnlineData a17;
    OnlineData b17;
    OnlineData c17;
	OnlineData yaw17;

	OnlineData obx18;
    OnlineData oby18;
    OnlineData obz18;
    OnlineData a18;
    OnlineData b18;
    OnlineData c18;
	OnlineData yaw18;

	OnlineData obx19;
    OnlineData oby19;
    OnlineData obz19;
    OnlineData a19;
    OnlineData b19;
    OnlineData c19;
	OnlineData yaw19;

	OnlineData obx20;
    OnlineData oby20;
    OnlineData obz20;
    OnlineData a20;
    OnlineData b20;
    OnlineData c20;
	OnlineData yaw20;


	ocp.setNOD(140);
	ocp.subjectTo(pow((x-obx1)*cos(yaw1)+(y-oby1)*sin(yaw1), 2)/pow(a1,2) + pow(-(x-obx1)*sin(yaw1)+(y-oby1)*cos(yaw1), 2)/pow(b1,2) + pow((z-obz1), 2)/pow(c1,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx2)*cos(yaw2)+(y-oby2)*sin(yaw2), 2)/pow(a2,2) + pow(-(x-obx2)*sin(yaw2)+(y-oby2)*cos(yaw2), 2)/pow(b2,2) + pow((z-obz2), 2)/pow(c2,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx3)*cos(yaw3)+(y-oby3)*sin(yaw3), 2)/pow(a3,2) + pow(-(x-obx3)*sin(yaw3)+(y-oby3)*cos(yaw3), 2)/pow(b3,2) + pow((z-obz3), 2)/pow(c3,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx4)*cos(yaw4)+(y-oby4)*sin(yaw4), 2)/pow(a4,2) + pow(-(x-obx4)*sin(yaw4)+(y-oby4)*cos(yaw4), 2)/pow(b4,2) + pow((z-obz4), 2)/pow(c4,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx5)*cos(yaw5)+(y-oby5)*sin(yaw5), 2)/pow(a5,2) + pow(-(x-obx5)*sin(yaw5)+(y-oby5)*cos(yaw5), 2)/pow(b5,2) + pow((z-obz5), 2)/pow(c5,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx6)*cos(yaw6)+(y-oby6)*sin(yaw6), 2)/pow(a6,2) + pow(-(x-obx6)*sin(yaw6)+(y-oby6)*cos(yaw6), 2)/pow(b6,2) + pow((z-obz6), 2)/pow(c6,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx7)*cos(yaw7)+(y-oby7)*sin(yaw7), 2)/pow(a7,2) + pow(-(x-obx7)*sin(yaw7)+(y-oby7)*cos(yaw7), 2)/pow(b7,2) + pow((z-obz7), 2)/pow(c7,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx8)*cos(yaw8)+(y-oby8)*sin(yaw8), 2)/pow(a8,2) + pow(-(x-obx8)*sin(yaw8)+(y-oby8)*cos(yaw8), 2)/pow(b8,2) + pow((z-obz8), 2)/pow(c8,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx9)*cos(yaw9)+(y-oby9)*sin(yaw9), 2)/pow(a9,2) + pow(-(x-obx9)*sin(yaw9)+(y-oby9)*cos(yaw9), 2)/pow(b9,2) + pow((z-obz9), 2)/pow(c9,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx10)*cos(yaw10)+(y-oby10)*sin(yaw10), 2)/pow(a10,2) + pow(-(x-obx10)*sin(yaw10)+(y-oby10)*cos(yaw10), 2)/pow(b10,2) + pow((z-obz10), 2)/pow(c10,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx11)*cos(yaw11)+(y-oby11)*sin(yaw11), 2)/pow(a11,2) + pow(-(x-obx11)*sin(yaw11)+(y-oby11)*cos(yaw11), 2)/pow(b11,2) + pow((z-obz11), 2)/pow(c11,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx12)*cos(yaw12)+(y-oby12)*sin(yaw12), 2)/pow(a12,2) + pow(-(x-obx12)*sin(yaw12)+(y-oby12)*cos(yaw12), 2)/pow(b12,2) + pow((z-obz12), 2)/pow(c12,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx13)*cos(yaw13)+(y-oby13)*sin(yaw13), 2)/pow(a13,2) + pow(-(x-obx13)*sin(yaw13)+(y-oby13)*cos(yaw13), 2)/pow(b13,2) + pow((z-obz13), 2)/pow(c13,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx14)*cos(yaw14)+(y-oby14)*sin(yaw14), 2)/pow(a14,2) + pow(-(x-obx14)*sin(yaw14)+(y-oby14)*cos(yaw14), 2)/pow(b14,2) + pow((z-obz14), 2)/pow(c14,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx15)*cos(yaw15)+(y-oby15)*sin(yaw15), 2)/pow(a15,2) + pow(-(x-obx15)*sin(yaw15)+(y-oby15)*cos(yaw15), 2)/pow(b15,2) + pow((z-obz15), 2)/pow(c15,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx16)*cos(yaw16)+(y-oby16)*sin(yaw16), 2)/pow(a16,2) + pow(-(x-obx16)*sin(yaw16)+(y-oby16)*cos(yaw16), 2)/pow(b16,2) + pow((z-obz16), 2)/pow(c16,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx17)*cos(yaw17)+(y-oby17)*sin(yaw17), 2)/pow(a17,2) + pow(-(x-obx17)*sin(yaw17)+(y-oby17)*cos(yaw17), 2)/pow(b17,2) + pow((z-obz17), 2)/pow(c17,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx18)*cos(yaw18)+(y-oby18)*sin(yaw18), 2)/pow(a18,2) + pow(-(x-obx18)*sin(yaw18)+(y-oby18)*cos(yaw18), 2)/pow(b18,2) + pow((z-obz18), 2)/pow(c18,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx19)*cos(yaw19)+(y-oby19)*sin(yaw19), 2)/pow(a19,2) + pow(-(x-obx19)*sin(yaw19)+(y-oby19)*cos(yaw19), 2)/pow(b19,2) + pow((z-obz19), 2)/pow(c19,2) -1.0 + sk_d >=  0 );
	ocp.subjectTo(pow((x-obx20)*cos(yaw20)+(y-oby20)*sin(yaw20), 2)/pow(a20,2) + pow(-(x-obx20)*sin(yaw20)+(y-oby20)*cos(yaw20), 2)/pow(b20,2) + pow((z-obz20), 2)/pow(c20,2) -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx3)*cos(yaw3), 2)/a3 + pow((y-oby3), 2)/b3 + pow((z-obz3), 2)/c3 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx4)*cos(yaw4), 2)/a4 + pow((y-oby4), 2)/b4 + pow((z-obz4), 2)/c4 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx5), 2)/a5 + pow((y-oby5), 2)/b5 + pow((z-obz5), 2)/c5 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx6), 2)/a6 + pow((y-oby6), 2)/b6 + pow((z-obz6), 2)/c6 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx7), 2)/a7 + pow((y-oby7), 2)/b7 + pow((z-obz7), 2)/c7 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx8), 2)/a8 + pow((y-oby8), 2)/b8 + pow((z-obz8), 2)/c8 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx9), 2)/a9 + pow((y-oby9), 2)/b9 + pow((z-obz9), 2)/c9 -1.0 + sk_d >=  0 );
	// ocp.subjectTo(pow((x-obx10), 2)/a10 + pow((y-oby10), 2)/b10 + pow((z-obz10), 2)/c10 -1.0 + sk_d >=  0 );
	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        90              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
	mpc.set( HOTSTART_QP,                 YES         );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          BT_FALSE             );
	mpc.set( GENERATE_MAKE_FILE,          BT_FALSE            );
	mpc.set( GENERATE_MATLAB_INTERFACE,   BT_FALSE             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, BT_FALSE             );
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, YES);

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "./src/CERLAB-UAV-Autonomy/trajectory_planner/include/trajectory_planner/mpc_solver" ) != SUCCESSFUL_RETURN){
		exit( EXIT_FAILURE );
	}
		

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}