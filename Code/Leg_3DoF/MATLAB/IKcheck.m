%%IKcheck, compare torque output for inverse formulation w simulated torq

% Run simulation     
sim = Leg_3DoF_ACA_jumpref_simulator;
sim.params.tspan = [0 3];
sim.run;

%%
% Get simulation data
tau         = sim.data.tau';

% Recreate q_ref matrix
t = 0:sim.params.Ts:sim.params.tspan(2);

q_ref = zeros(length(t),6);
q_d_ref = zeros(length(t),6);

for k = 1:length(t)
     [q_ref(k,:),q_d_ref(k,:)] = sim.model.q_ref(t(k)); % q_ref = [q_ref, q_d_ref]
end

%% 
%Preallocate
        qp_ddt_act = zeros(length(t),6);
        
for k=1:length(t)
    % State and input
    x = [q_ref(k,:)'; q_d_ref(k,:)'];
    u(:,k) = [tau(k,1); tau(k,2); tau(k,3)];

    % No ext forces
    Fe = zeros(3,1);
    
    % Ground Force F_GRF
    % Leg: q = [x1, y1, theta, q1, q2, q3]
    % Leg: x = [q; q_d]
    q_leg   = x(1 : sim.model.leg.N/2); %#ok<*NASGU>
    q_leg_d = x(sim.model.leg.N/2 + 1 : sim.model.leg.N);
    
    % Build ground reaction force (GRF)
    % The floor is a spring-damper in the y-direction, and
    % viscous+Coulomb friction in the x-direction.
    % F_GRF = [F_heel_x; F_heel_y; F_toe_x; F_toe_y]
    [~, y_heel, ~]              = sim.model.leg.calc_fwdKin_named(q_leg, 'heel');
    [~, y_toe, ~]               = sim.model.leg.calc_fwdKin_named(q_leg, 'toe');
    [x_ankle_d, y_ankle_d, ~]   = sim.model.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'heel');
    [x_toe_d, y_toe_d, ~]       = sim.model.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'toe');
     F_ankle_x   = 0;
     F_ankle_y   = 0;
     F_toe_x     = 0;
     F_toe_y     = 0;
     floor_K     = sim.model.leg.params.floor_K;
     floor_D     = sim.model.leg.params.floor_D;
        if (y_heel <= 0)
            F_ankle_y   = floor_K * (0.0 - y_heel) + max(0, floor_D * (0.0 - y_ankle_d));
            coulomb     = -sim.model.leg.params.coulomb * tanh(500*x_ankle_d);
            F_ankle_x   = coulomb + F_ankle_y * sim.model.leg.params.mu * (0.0 - x_ankle_d);
        end
        if (y_toe <= 0)
            F_toe_y     = floor_K * (0.0 - y_toe) + max(0, floor_D * (0.0 - y_toe_d));
            coulomb     = -sim.model.leg.params.coulomb * tanh(500*x_toe_d);
             F_toe_x     = coulomb + F_toe_y * sim.model.leg.params.mu * (0.0 - x_toe_d);
        end
        
        F_GRF = [F_ankle_x; F_ankle_y; F_toe_x; F_toe_y];
            
    
    % State evaluation to obtain active joint accelerations
     x_d = sim.model.leg.dx(t(k), x, u(:,k), F_GRF, Fe); %x_d     = [q_d; q_dd];
    
    % Active joint accelerations
    qa_dd = x_d(10:12);

    % Inverse formulation

    % M-matrices with  M = [Mpp Mpa ; Map Maa]
    M  = sim.model.leg.calc_M(q_ref(k,:)); %6x6
    Mpp = M(1:3,1:3);
    Mpa = M(1:3,4:6);
    Map = M(4:6,1:3);
    Maa = M(4:6,4:6);
    % New formulation Ma (to be inverted)
    Ma=[Mpp zeros(3,3); Map -diag(ones(1,3))];
    % New formulation Mb (rhs)
    Mb = [Mpa;Maa];

    
    
    % Other terms
    G = sim.model.leg.calc_G(q_ref(k,:)'); 
    C = sim.model.leg.calc_C(q_ref(k,:)',q_d_ref(k,:)'); %
    J_GRF   = sim.model.leg.calc_J_GRF(q_ref(k,:)');
    D = sim.model.leg.params.d; 
    % B = - G + C + D; 

    % Inverse formulation
    qp_ddt_act(k,:) = Ma \ ( -Mb * qa_dd + G - D .* q_d_ref(k,:)' - C * q_d_ref(k,:)' + J_GRF' * F_GRF ); 

end

qp_dd = qp_ddt_act(:,1:3); % passive accelerations from inverse formulation
t_act = qp_ddt_act(:,4:6);  %Torque active from inverse formulation

%% 
subplot(3,1,1)
plot(t,tau(:,1),'b',t,t_act(:,1),'r--');
legend('\tau simulated','\tau inverse');
title('\tau_1');
subplot(3,1,2)
plot(t,tau(:,2),'b',t,t_act(:,2),'r--')
legend('\tau simulated','\tau inverse');
title('\tau_2');
subplot(3,1,3)
plot(t,tau(:,3),'b',t,t_act(:,3),'r--')
legend('\tau simulated','\tau inverse');
title('\tau_3');