%% Set optimizers and parameters

% No Esb
noESB   = Leg_3DoF_ACA_jumpref_optimizer;

% Mono
mono    = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_monoarticulated.mat','Leg_3DoF_design_mono');

% Bi
bi      = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_biarticulated.mat','Leg_3DoF_design_bi');

%% Load data

load noESB.mat
noESB.load_matdata

load mono
mono.load_matdata

load bi.mat
bi.load_matdata

disp(['noESB c_torq = ',num2str(noESB.params.c_torq)])
disp(['mono  c_torq = ',num2str(mono.params.c_torq)])
disp(['bi    c_torq = ',num2str(bi.params.c_torq)])

%% Post-process 

% Save optimization data noESB
% optimization_data   = com_noESB.data;
% simulation_data     = com_noESB.sim.data;
% save(['comp_noESB','optimization_data','simulation_data'])

% Calculate CoM_y noESB
noESB.simulate_solution;
[~,~,~,~, CoM_y_noESB,~] = noESB.Calc_IK;
disp(['CoM_y_noESB = ',num2str(CoM_y_noESB)]);

% Save optimization data mono
% optimization_data   = com_mono.data;
% simulation_data     = com_mono.sim.data;
% save('comp_mono','optimization_data','simulation_data')

% Calculate CoM_y ,mono
mono.simulate_solution;
[~,~,~,~, CoM_y_mono,~] = mono.Calc_IK;
disp(['CoM_y_mono = ',num2str(CoM_y_mono)]);

% Save optimization data bi
% optimization_data   = com_bi.data;
% simulation_data     = com_bi.sim.data;
% save('comp_bi','optimization_data','simulation_data')


% Calculate CoM_y bi
bi.simulate_solution;

[~,~,~,~, CoM_y_bi,~] = bi.Calc_IK;
disp(['CoM_y_bi = ',num2str(CoM_y_bi)]);

fprintf('\n');
disp(['CoM_y_noESB = ',num2str(CoM_y_noESB)]);
disp(['CoM_y_mono  = ',num2str(CoM_y_mono)]);
disp(['CoM_y_bi    = ',num2str(CoM_y_bi)]);



% CoM_y_noESB = 0.91667
% CoM_y_mono  = 0.92433
% CoM_y_bi    = 0.99311



