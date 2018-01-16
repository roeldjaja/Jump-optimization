%% Set optimizers and parameters

% No Esb
com_noESB   = Leg_3DoF_ACA_jumpref_optimizer;

% Mono
com_mono    = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_monoarticulated.mat','Leg_3DoF_design_mono');

% Bi
com_bi      = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_biarticulated.mat','Leg_3DoF_design_bi');

%% Run optimizations

com_noESB.run
com_mono.run
com_bi.run

%% Post-process 



% Save optimization data noESB
% optimization_data   = com_noESB.data;
% simulation_data     = com_noESB.sim.data;
% save(['comp_noESB','optimization_data','simulation_data'])

% Calculate CoM_y noESB
com_noESB.simulate_solution;
[~,~,~,~, CoM_y_noESB,~] = com_noESB.Calc_IK;
disp(['CoM_y_noESB = ',num2str(CoM_y_noESB)]);

% Save optimization data mono
% optimization_data   = com_mono.data;
% simulation_data     = com_mono.sim.data;
% save('comp_mono','optimization_data','simulation_data')

% Calculate CoM_y ,mono
com_mono.simulate_solution;
[~,~,~,~, CoM_y_mono,~] = com_mono.Calc_IK;
disp(['CoM_y_mono = ',num2str(CoM_y_mono)]);

% Save optimization data bi
% optimization_data   = com_bi.data;
% simulation_data     = com_bi.sim.data;
% save('comp_bi','optimization_data','simulation_data')

% Calculate CoM_y bi
com_bi.simulate_solution;
[~,~,~,~, CoM_y_bi,~] = com_bi.Calc_IK;
disp(['CoM_y_bi = ',num2str(CoM_y_bi)]);
















