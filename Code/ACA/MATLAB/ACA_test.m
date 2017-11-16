% ACA_test.m
% Test an ACA model in simulation

function [t, x, data] = ACA_test()

    % Get ACA object
    a = ACA('ACA_test_params');

    % Parameters
    x0      = [zeros(a.N,1); 0; 0];     % Initial conditions [[x_ACA], [x_mass]]
    tspan	= [0 1];                   % Simulation time [s]

    % Set topology
    a.setTopology(-1, 1);


    %% Simulate

    % Set state to initial state
    x = x0;

    % ode45 options
    odeopt = odeset(    'InitialStep', 1e-3, ...
                        'MaxStep', 1e-1, ...
                        'RelTol', 1e-3, ...
                        'AbsTol', 1e-6, ...
                        'OutputFcn', @outFun        );

    tic

    % Simulate
    data = [];
    [t, x] = ode45(@(t,x) ACA_test_dynamics(a,t,x), tspan, x0, odeopt);

    % Get the ACA and mass states
    x_ACA	= x(:,1:end-2)';
    x_mass	= x(:,end-1:end)';

    % Get SS for state/output names
    sys = a.getStateSpace();

    toc


    %% Plot results
    
    figure(1); clf; hold on; grid on;
    plot(t,x_mass(1,:));
    plot(t,x_mass(2,:));
    legend('x', 'x_d');
    title('Mass state');

    figure(2); clf; hold on; grid on;
    plot(t,x_ACA);
    legend(sys.StateName);
    title('ACA state');
    
    
    %% Output function to save intermediate data
    function stop = outFun(t, x, flag) %#ok<*INUSD>
        stop = 0;
        data = [data, x];
        
        % Return for init, empty x and draw only every 0.1s
        if (    ~isequal(flag, 'init') || ...
                isempty(x) || ...
                floor(10*t(1)) == floor(10*t(end))      )
            return;
        end
        
        figure(1337); clf;
        bar(x(:,end));
        title(['t = ' num2str(t(end),2) ' s.']);
        drawnow;
    end
    
end