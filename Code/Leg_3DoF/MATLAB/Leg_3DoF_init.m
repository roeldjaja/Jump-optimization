% Leg_3DoF_init.m
% Load the required paths and initialise parameters for the Leg_3DoF package

% Include directory for common functions
addpath(genpath('include'));

% Include library folder
addpath(genpath('lib'));

% Include parameters dir
addpath(genpath('params'));

% Include projects
addpath(genpath('Leg_3DoF_ACA_optimizer'));
addpath(genpath('Leg_3DoF_ACA_simulator'));
addpath(genpath('Leg_3DoF_interactive'));

addpath(genpath('Leg_3DoF_noAct_simulator'));
addpath(genpath('Leg_3DoF_ACA_jumpref_simulator'));
addpath(genpath('Leg_3DoF_ACA_jumpref_optimizer'));

% Dependency on ACA class
addpath(genpath(['..' filesep '..' filesep 'ACA/MATLAB']));

% fastBspline functions
addpath(genpath(['..' filesep '..' filesep 'external/fastbspline']));

