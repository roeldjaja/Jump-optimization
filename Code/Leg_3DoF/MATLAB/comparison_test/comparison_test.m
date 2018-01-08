%% Set optimizers and parameters

% Optimoptions
InitTrustRegionRadius    = 1;  
DiffMinChange            = 1e-3;
DiffMaxChange            = 1;
            
% Performance (high)
c_high  = 1;            
% Energy
c_ener  = 1;            
% Stability
c_xh    = 1e4;      
c_xm    = 1e2;            
% Torque
c_torq  =  1.e-08;            
% Control point parameters 
cpres   = 200;

% No Esb
com_noESB   = comparison_optimizer;

com_noESB.params.InitTrustRegionRadius    = InitTrustRegionRadius;   
com_noESB.params.DiffMinChange            = DiffMinChange;
com_noESB.params.DiffMaxChange            = DiffMaxChange; 

% com_noESB.sim.model.p    = p;
com_noESB.params.c_high  = c_high;            
com_noESB.params.c_ener  = c_ener;            
com_noESB.params.c_xh    = c_xh;      
com_noESB.params.c_xm    = c_xm;                
com_noESB.params.c_torq  = c_torq;            
com_noESB.params.cpres   = cpres;

% Mono
com_mono    = comparison_optimizer('actuatorParams_monoarticulated.mat','Leg_3DoF_design_mono');

com_mono.params.InitTrustRegionRadius    = InitTrustRegionRadius;   
com_mono.params.DiffMinChange            = DiffMinChange;
com_mono.params.DiffMaxChange            = DiffMaxChange; 

% com_mono.sim.model.p    = p;
com_mono.params.c_high  = c_high;            
com_mono.params.c_ener  = c_ener;            
com_mono.params.c_xh    = c_xh;      
com_mono.params.c_xm    = c_xm;                
com_mono.params.c_torq  = c_torq;            
com_mono.params.cpres   = cpres;

% Bi
com_bi      = comparison_optimizer('actuatorParams_biarticulated.mat','Leg_3DoF_design_bi');
com_bi.sim.model.p

com_bi.params.InitTrustRegionRadius    = InitTrustRegionRadius;   
com_bi.params.DiffMinChange            = DiffMinChange;
com_bi.params.DiffMaxChange            = DiffMaxChange; 

% com_bi.sim.model.p    = p;0.01
com_bi.params.c_high  = c_high;            
com_bi.params.c_ener  = c_ener;            
com_bi.params.c_xh    = c_xh;      
com_bi.params.c_xm    = c_xm;                
com_bi.params.c_torq  = c_torq;            
com_bi.params.cpres   = cpres;

%% Run optimizations
com_noESB.run
com_mono.run
com_bi.run

%% Post-process 

% Save optimization data noESB
optimization_data   = com_noESB.data;
simulation_data     = com_noESB.sim.data;
save('comp_noESB','optimization_data','simulation_data')

% Calculate CoM_y noESB
com_noESB.simulate_solution;
[~,~,~,~, CoM_y_noESB,~] = com_noESB.Calc_IK;

% Save optimization data mono
optimization_data   = com_mono.data;
simulation_data     = com_mono.sim.data;
save('comp_mono','optimization_data','simulation_data')

% Calculate CoM_y ,mono
com_mono.simulate_solution;
[~,~,~,~, CoM_y_mono,~] = com_mono.Calc_IK;

% Save optimization data bi
optimization_data   = com_bi.data;
simulation_data     = com_bi.sim.data;
save('comp_bi','optimization_data','simulation_data')

% Calculate CoM_y bi
com_bi.simulate_solution;
[~,~,~,~, CoM_y_bi,~] = com_bi.Calc_IK;

















