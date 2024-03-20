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
 * ACADO OCP solver wrapper.
 */

#include <string.h>
#include "acado_solver_sfunction.h"
#include "acado_auxiliary_functions.h"

/* Make instances of ACADO structures. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* Initialized indicator. */
static unsigned initialized = 0;

void acado_step(ACADOinput* inData, ACADOoutput* outData)
{
    #if !(defined _DSPACE)
	acado_timer tmr;
	
	/* Start measuring time. */
	acado_tic( &tmr );
    #endif
	
	/* Copy all outside world data here. */	
	memcpy(&acadoVariables.x, &(inData->data), sizeof( ACADOdata ));  /* offset necessary because of the dummy variable ! */
	
	/* Call solver */
	switch ( inData->control )
	{
		case 0:
			/* Simple operational mode. Run one RTI with optional shifting. */
			
			if ( !initialized )
			{				
				acado_initialize( );
				if (inData->initialization == 1)
				{
					acado_initializeNodesByForwardSimulation();
				}
				
				initialized = 1;
			}
			else if (inData->shifting == 1 || inData->shifting == 2)
			{
			
#if ACADO_QP_SOLVER == ACADO_QPDUNES
				acado_shiftQpData();
#endif
			
				acado_shiftStates(inData->shifting, 0, 0);
				acado_shiftControls( 0 );
			}
			
			acado_preparationStep();
			
			outData->status = acado_feedbackStep();
			
			outData->kktValue = acado_getKKT();
			outData->objValue = acado_getObjective();
			
#if ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
 			outData->nIterations = acado_getNWSR();
#endif /* ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) ) */
			
			break;
		
		case 1:
			/* Initialize */
				
			acado_initialize( );
			if (inData->initialization == 1)
			{
				acado_initializeNodesByForwardSimulation();
			}
			
			initialized = 1;
			
			break;
		
		case 2:
			/* Preparation step */
			
			acado_preparationStep();
			
			break;
		
		case 3:
			/* Feedback step */
			
			outData->status = acado_feedbackStep();
			
			outData->kktValue = acado_getKKT();
			outData->objValue = acado_getObjective();
				
#if ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
			outData->nIterations = acado_getNWSR();
#endif /* ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) ) */
						
			break;
		
		case 4:
			/* Shifting */
			
#if ACADO_QP_SOLVER == ACADO_QPDUNES
			acado_shiftQpData();
#endif
			
			acado_shiftStates(inData->shifting, 0, 0);
			acado_shiftControls( 0 );
			
			break;
			
		default:
			/* Return default error code */
			outData->status = -1;
	}
	
	/* Copy the data to outside world. */
	memcpy(&(outData->data), &acadoVariables.x, sizeof( ACADOdata ));   /* offset necessary because of the dummy variable ! */
	
    #if !(defined _DSPACE)
	/* Read the elapsed time. */
	outData->execTime = acado_toc( &tmr );
    #endif
}

void acado_initialize( void )
{
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));	
	acado_initializeSolver();
	initialized = 0;
}

void acado_cleanup( void )
{
#if ACADO_QP_SOLVER == ACADO_QPDUNES
	/* Clean-up of the qpDUNES QP solver. */
	acado_cleanupSolver();
#endif /* ACADO_QP_SOLVER == ACADO_QPDUNES */
}
