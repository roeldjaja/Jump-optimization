% ACA_test.m
% Test an ACA model in simulation

% Get ACA object
a = ACA('ACA_test_params');

% Modify topology
a.setTopology(-0.05, 1);

% Simulation parameters
u       = [1; 1; 0];     	% Input
tspan	= [0 10];           % Simulation time span


%% Simulate

% ode45 options
odeopt = odeset(    'InitialStep', 1e-4, ...
                    'RelTol', 1e-3, ...
                    'AbsTol', 1e-3              );
                
% Simulate
[t, x] = ode45(@(t,x) a.dx(t,x,u), tspan, a.x0, odeopt);


%% Plotting

% Plot states
figure(1); clf; hold on; grid on;
plot(t,x);
sys = a.getStateSpace();
legend(sys.StateName);
title('States');

% Calculate outputs
y = a.y(t, x', repmat(u,[1 length(t)]));

% Plot outputs
figure(2); clf; hold on; grid on;
plot(t,y);
legend(sys.OutputName);
title('Outputs');