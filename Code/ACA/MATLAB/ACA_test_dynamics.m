function [ dx ] = ACA_test_dynamics( ACAObj, t, x )

    % x = [[x_ACA]; [x_mass]]
    % u = [[u_ACA]; u_mass] = [[u_ACA]; F_ext]

    
    %% Get states
    
    % Get the ACA and mass states
    stateIdx_ACA	= 1:length(x)-2;
    stateIdx_mass	= length(x)-1:length(x);
    x_ACA           = x(stateIdx_ACA);
    x_mass          = x(stateIdx_mass);
    
    
    %% Get outputs
    
    % Get the ACA output
    y_ACA = ACAObj.y(t, x_ACA, [0; 0; x_mass(2)]);
    
    % Get the mass output (simply its state)
    %y_mass = x_mass;
    
    
    %% Calculate/get new inputs
    
    % Simple PD control law on mass
    tau_ref = 10*(0 - x_mass(1)) + 5*(x_mass(2));
    %tau_ref = 100; % For step response
    
    % Calculate ACA current inputs
    [i_1, i_2] = ACAObj.controller(t, x_ACA, tau_ref, x_mass(1), x_mass(2), 0, 0);
    
    % Build the ACA input
    % Connect the mass's velocity to the ACA input
    % u_ACA = [i1, i2, q_d]
    % x_mass = [x, x_d]
    u_ACA = [i_1; i_2; x_mass(2)];
    
    % Build the mass input
    % u_mass = F = tau + F_ext
    u_mass = y_ACA(ACAObj.outputIdx_tau) - 9.81;
    
    %disp([  'dL_p = ' num2str(x_ACA(4)) ...
    %        ', tau = ' num2str(y_ACA(ACAObj.outputIdx_tau)) ...
    %        ', F_ext = ' num2str(u(3))]     );
    
    
    %% Get dynamics
    
    % Get the ACA dynamics
    dx_ACA = ACAObj.dx(t, x_ACA, u_ACA);
    
    % Get the mass dynamics
    I = 1.0; % Inertia
    % x_mass = [x, x_d]
    dx_mass = [x_mass(2); (u_mass - 0.1 * x_mass(2)) / I];
    %dx_mass = [0; 0]; % To fix mass
    
    % Assemble state derivative
    dx = [dx_ACA; dx_mass];

end

