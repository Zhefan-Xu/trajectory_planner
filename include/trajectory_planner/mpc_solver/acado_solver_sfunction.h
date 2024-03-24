/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


/*
 * ACADO OCP solver wrapper header.
 */
 
#ifndef ACADO_SOLVER_SFUNCTION
#define ACADO_SOLVER_SFUNCTION

/* Include the common header. */
#include "acado_common.h"

/** Internal data structure for communicating with the solver
  * NOTE: Do not change order of the fields here
  */
typedef struct
{
	real_t x[(ACADO_N + 1) * ACADO_NX];
	
#if ACADO_NXA
	real_t z[ACADO_N * ACADO_NXA];
#endif

	real_t u[ACADO_N * ACADO_NU];

#if ACADO_NOD
	real_t od[(ACADO_N + 1) * ACADO_NOD];
#endif

	real_t y[ACADO_N * ACADO_NY];
	real_t yN[ACADO_NYN];
	
#if ACADO_WEIGHTING_MATRICES_TYPE == 1
	real_t W[ACADO_NY * ACADO_NY];
	real_t WN[ACADO_NYN * ACADO_NYN];
#elif ACADO_WEIGHTING_MATRICES_TYPE == 2
	real_t W[ACADO_N * ACADO_NY * ACADO_NY];
	real_t WN[ACADO_NYN * ACADO_NYN];
#endif

#if ACADO_USE_ARRIVAL_COST == 1
	real_t xAC[ ACADO_NX ];
	real_t SAC[ACADO_NX * ACADO_NX];
	real_t WL[ACADO_NX * ACADO_NX];
#endif
	
#if ACADO_INITIAL_STATE_FIXED
	real_t x0[ ACADO_NX ];
#endif /* #if ACADO_INITIAL_STATE_FIXED */

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
	real_t sigmaN[ACADO_NX * ACADO_NX];
#endif

} ACADOdata;

/** Input data structure for the ACADO OCP solver. */
typedef struct
{
	int control;			/**< Control flag. */
	int shifting; 			/**< Shifting strategy. */
	int initialization;		/**< Initialization flag. */
	ACADOdata data;	        /**< Input data. */
} ACADOinput;

/** Output data structure for the ACADO OCP solver. */
typedef struct
{
	int status;				/**< Status (ATM from the QP solver). */
	int nIterations;        /**< @ iteration of the QP solver. */
	real_t kktValue;		/**< KKT value. */
	real_t objValue;		/**< Objective value. */
	real_t execTime;		/**< Execution time. */
	ACADOdata data;	        /**< Output data. */
} ACADOoutput;

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/** The step function of the solver. */
void acado_step(
	ACADOinput*  inData,	/**< Input data. */
	ACADOoutput* outData	/**< Output data. */
	);

/** Initialization of the solver. */
void acado_initialize( void );

/** Cleanup of the solver. */
void acado_cleanup( void );

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* acado_SOLVER_SFUNCTION */
