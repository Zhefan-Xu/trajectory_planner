%
%    This file was auto-generated using the ACADO Toolkit.
%    
%    While ACADO Toolkit is free software released under the terms of
%    the GNU Lesser General Public License (LGPL), the generated code
%    as such remains the property of the user who used ACADO Toolkit
%    to generate this code. In particular, user dependent data of the code
%    do not inherit the GNU LGPL license. On the other hand, parts of the
%    generated code that are a direct copy of source code from the
%    ACADO Toolkit or the software tools it is based on, remain, as derived
%    work, automatically covered by the LGPL license.
%    
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%    


%% Legacy code wrapper for the ACADO CGT generated OCP solver

% Dimensions
ACADO_N   = 30;
ACADO_NX  = 7;
ACADO_NDX = 0;
ACADO_NXA = 0;
ACADO_NU  = 4;
ACADO_NOD = 60;
ACADO_NY  = 7;
ACADO_NYN = 3;

ACADO_QP_SOLVER                   = 'QPOASES';
ACADO_INITIAL_STATE_FIXED         = 1;
ACADO_WEIGHTING_MATRICES_TYPE     = 0;
ACADO_HARDCODED_CONSTRAINT_VALUES = 1;
ACADO_USE_ARRIVAL_COST            = 0;
ACADO_COMPUTE_COVARIANCE_MATRIX   = 0;

aSfunName  = 'acado_solver_sfun';
aRealT     = 'double';
aHeaderFile = 'acado_solver_sfunction.h';

%% Define busses

clear avCells aInput aOutput ACADOdata ACADOinput ACADOoutput;

% Define ACADOvariables bus:
avCells = {'ACADOdata', aHeaderFile, '', 'Auto', '-1', {}};

idx = 1;
avCells{ 6 }{ idx } = {'x', [1, (ACADO_N + 1) * ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1; 
avCells{ 6 }{ idx } = {'u', [1, ACADO_N * ACADO_NU], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;

if ACADO_NXA > 0
    avCells{ 6 }{ idx } = {'z', [1, ACADO_N * ACADO_NXA], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

if ACADO_NOD > 0
    avCells{ 6 }{ idx } = {'od', [1, (ACADO_N + 1) * ACADO_NOD], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

avCells{ 6 }{ idx } = {'y', [1, ACADO_N * ACADO_NY], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1; 
avCells{ 6 }{ idx } = {'yN', [1, ACADO_NYN], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;

if ACADO_WEIGHTING_MATRICES_TYPE == 1
    avCells{ 6 }{ idx } = {'W', [1, ACADO_NY * ACADO_NY], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
elseif ACADO_WEIGHTING_MATRICES_TYPE == 2
    avCells{ 6 }{ idx } = {'W', [1, ACADO_NY * ACADO_N * ACADO_NY], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

if ACADO_WEIGHTING_MATRICES_TYPE ~= 0
    avCells{ 6 }{ idx } = {'WN', [1, ACADO_NYN * ACADO_NYN], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

if ACADO_USE_ARRIVAL_COST == 1
	avCells{ 6 }{ idx } = {'xAC', [1, ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
	avCells{ 6 }{ idx } = {'SAC', [1, ACADO_NX * ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
	avCells{ 6 }{ idx } = {'WL', [1, ACADO_NX * ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

if ACADO_INITIAL_STATE_FIXED == 1
    avCells{ 6 }{ idx } = {'x0', [1, ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
    avCells{ 6 }{ idx } = {'sigmaN', [1, ACADO_NX * ACADO_NX], aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; idx = idx + 1;
end;

% Define input bus for the Simulink component:
aInput = { ...
	'ACADOinput', aHeaderFile, '', ...
	{
		{'control', 1, 'int32', -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'shifting', 1, 'int32', -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'initialization', 1, 'int32', -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'data', 1, 'ACADOdata', -1, 'real', 'Sample', 'Fixed', [], []}; ...
	}
	};

% Define output bus for the Simulink component:
aOutput = { ...
	'ACADOoutput', aHeaderFile, '', ...
	{
		{'status', 1, 'int32', -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'kktValue', 1, aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'objValue', 1, aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'execTime', 1, aRealT, -1, 'real', 'Sample', 'Fixed', [], []}; ...
		{'data', 1, 'ACADOdata', -1, 'real', 'Sample', 'Fixed', [], []}; ...
	}
	};

% Generate all structures
Simulink.Bus.cellToObject( {avCells, aInput, aOutput} );

%% Define S-function wrapper

clear ACADODef;

ACADODef = legacy_code('initialize');

if strcmp(ACADO_QP_SOLVER, 'QPOASES')
	ACADODef.Options.language = 'C++';
elseif strcmp(ACADO_QP_SOLVER, 'QPOASES3')
	ACADODef.Options.language = 'C';
elseif strcmp(ACADO_QP_SOLVER, 'QPDUNES')
	ACADODef.Options.language = 'C';
elseif strcmp(ACADO_QP_SOLVER, 'HPMPC')
	ACADODef.Options.language = 'C';
else
	error('Unknown QP solver')
end;


% Define the S-function name
ACADODef.SFunctionName = aSfunName;

% Define source files
ACADODef.SourceFiles = { ...
	'acado_solver.c', ...
	'acado_integrator.c', ...
	'acado_auxiliary_functions.c', ...
	'acado_solver_sfunction.c' ...
	};
	
if strcmp(ACADO_QP_SOLVER, 'QPOASES')
	ACADODef.SourceFiles{end + 1} = 'acado_qpoases_interface.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/Bounds.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/Constraints.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/CyclingManager.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/Indexlist.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/MessageHandling.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/QProblem.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/QProblemB.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/SubjectTo.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/Utils.cpp';
	ACADODef.SourceFiles{end + 1} = 'qpoases/SRC/EXTRAS/SolutionAnalysis.cpp';
elseif strcmp(ACADO_QP_SOLVER, 'QPOASES3')
	ACADODef.SourceFiles{end + 1} = 'acado_qpoases3_interface.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Bounds.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Constraints.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Indexlist.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Matrices.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/MessageHandling.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Options.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Flipper.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/QProblem.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/QProblemB.c';
	ACADODef.SourceFiles{end + 1} = 'qpoases3/src/Utils.c';
elseif strcmp(ACADO_QP_SOLVER, 'QPDUNES')
	ACADODef.SourceFiles{end + 1} = 'qpdunes/src/dual_qp.c';
	ACADODef.SourceFiles{end + 1} = 'qpdunes/src/matrix_vector.c';
	ACADODef.SourceFiles{end + 1} = 'qpdunes/src/setup_qp.c';
	ACADODef.SourceFiles{end + 1} = 'qpdunes/src/stage_qp_solver_clipping.c';
	ACADODef.SourceFiles{end + 1} = 'qpdunes/src/utils.c';
elseif strcmp(ACADO_QP_SOLVER, 'HPMPC')
	ACADODef.SourceFiles{end + 1} = 'acado_hpmpc_interface.c';
else
	error('Unknown QP solver')
end;

% Define header files
ACADODef.HeaderFiles = {'acado_common.h', aHeaderFile};

% Define include folders
if strcmp(ACADO_QP_SOLVER, 'QPOASES')
	ACADODef.IncPaths = { ...
    	'qpoases', ...
		'qpoases/INCLUDE', ...
		'qpoases/SRC' ...
	};
elseif strcmp(ACADO_QP_SOLVER, 'QPOASES3')
	ACADODef.IncPaths = { ...
    	'qpoases3', ...
		'qpoases3/include', ...
		'qpoases3/src' ...
	};
elseif strcmp(ACADO_QP_SOLVER, 'QPDUNES')
	ACADODef.IncPaths = { ...
		'qpdunes/include', ...
	};
elseif strcmp(ACADO_QP_SOLVER, 'HPMPC')
	ACADODef.IncPaths = { ...
		'hpmpc/include', ...
	};
else
	error('Unknown QP solver')
end;

% link against some libs maybe
ldFlags = '';
if (isunix() && ~ismac())
%	ldFlags = [ldFlags '-lrt'];
elseif (ispc)
    ldFlags = [ldFlags '-DWIN32'];
end;

%add Hpmpc static library
if strcmp(ACADO_QP_SOLVER, 'HPMPC')
    ACADODef.LibPaths = {'hpmpc'};
    ACADODef.HostLibFiles = {'libhpmpc.a'};
end
% Add an extra option for qpOASES
iFlags = {''};
if (~strcmp(aSfunName, 'acado_solver_sfun')) && (strcmp(ACADO_QP_SOLVER, 'QPOASES'))
    iFlags = {'-DQPOASES_CUSTOM_INTERFACE="acado_qpoases_interface.hpp"'};    
end;
if strcmp(ACADO_QP_SOLVER, 'QPOASES3')
    iFlags = {'-D__CODE_GENERATION__', '-DQPOASES_CUSTOM_INTERFACE="acado_qpoases3_interface.h"'};    
end;
if strcmp(ACADO_QP_SOLVER, 'QPDUNES')
	iFlags = {'-D__SIMPLE_BOUNDS_ONLY__'};
end;
if strcmp(ACADO_QP_SOLVER, 'HPMPC')
	iFlags = '';
end;i

% Define output function declaration
ACADODef.OutputFcnSpec = [ ...
	'acado_step(' ... % Wrapper function name
	'ACADOinput u1[1], ' ...	% Input argument
	'ACADOoutput y1[1])' ...	% Output argument
	];
% Define init function declaration
ACADODef.StartFcnSpec     = 'acado_initialize( void )';
% Define terminate function declaration
ACADODef.TerminateFcnSpec = 'acado_cleanup( void )';

%% Generate S-function source file
legacy_code('sfcn_cmex_generate', ACADODef);

%% Compile the code
legacy_code('compile', ACADODef, {iFlags{:}, ldFlags});

%% Generate a TLC file and simulation block
legacy_code('slblock_generate', ACADODef);
legacy_code('sfcn_tlc_generate', ACADODef);
% Mandatory, because not all the source and header files are in the same folder
legacy_code('rtwmakecfg_generate', ACADODef);

%% Remove dependency on the header file and regenerate the Simulink structures

avCells{ 2 } = '';
aInput{ 2 } = '';
aOutput{ 2 } = '';

% Generate all structures
Simulink.Bus.cellToObject( {avCells, aInput, aOutput} );

%% Clear byproducts
clear aInput aOutput avCells idx iFlags ldFlags aSfunName aRealT aHeaderFile;
