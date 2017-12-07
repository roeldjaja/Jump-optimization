% Pretension test
% File to do test with pretension position (temporary)

% Load jump reference
load 061217_2

% Construct optimizer, simulator, model, leg
opt =  Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_biarticulated.mat', 'Leg_3DoF_design_bi');
plist = [];

CoM_xh = zeros(1,8);
CoM_xm = zeros(1,8);
CoM_xf = zeros(1,8);
CoM_y = zeros(1,8);

for k = 1:8
    opt.sim.model.p = -0.024 + (k-1)*0.0097;
    plist = [plist opt.sim.model.p];
    opt.list.F_GRF = [];
    
    % Set simulator jump reference
    opt.sim.model.ref.random.q_ref = optimization_data.q_res;
    opt.sim.model.ref.random.q_d_ref = optimization_data.q_d_res;
    opt.sim.model.ref.use_random =1;
    
    % Set initial states
    opt.sim.model.setInitialStates( opt.sim.model.ref.random.q_ref(:,1), opt.sim.model.ref.random.q_d_ref(:,1) )

    % Simulate with jump reference
    opt.sim.run(1);

    % Get max height
    [tau_IK, CoM_xh(k), CoM_xm(k), CoM_xf(k), CoM_y(k)] = opt.Calc_IK;
end

plot(plist,CoM_y,'*');title('Max CoM_y for different pretension positions (Bi-articulated)');
xlabel('p');ylabel('CoM_y');