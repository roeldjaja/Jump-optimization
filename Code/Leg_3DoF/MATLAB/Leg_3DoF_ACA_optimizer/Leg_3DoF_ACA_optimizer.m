classdef Leg_3DoF_ACA_optimizer < handle
    % Leg_3DoF_ACA_optimizer Class for optimization of ESB parameters of the
    % 3-DoF leg actuated by Asymmetric Compliant Actuators (ACAs)
    
    %__________________________________________________________________
    % Properties
    properties
        params      % Optimization parameters
        model       % Leg_3DoF_ACA object
        results     % Optimisation results
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_ACA_optimizer(legParamsName)
            % Default parameters
            if (~exist('legParamsName', 'var'))
                legParamsName = '';
            end
            
            % Set empty struct for results
            this.results = struct;
            
            % Get model
            % No need to load actuation parameters, because that's what
            % we're optimising here and they will be overwritten below.
            quiet = 1;
            this.model = Leg_3DoF_ACA('', legParamsName, quiet);
            
            % Define optimization range
            l2 = this.model.leg.params.l2; % l2 (shank) shorthand
            l3 = this.model.leg.params.l3; % l3 (thigh) shorthand
            this.params.r_limits	= [l2 l2+l3];	% Radial range (of hip) [m]
            this.params.beta_limits	= [-0.4 0.25];	% Angular range (of hip) [rad]
            this.params.checkPolar  = 0; % Whether to check the configuration's polar coordinates r,beta
            this.params.checkCoP    = 1; % Whether the CoP should be checked for each config.
            
            % Define parameter lower and upper bounds
            this.params.r_lb    = 0.01;   	% Pulley radius lower bound [m]
            this.params.r_ub    = 0.10;   	% Pulley radius upper bound [m]
            this.params.k_p_lb	= 2000.0;   % Stiffness lower bound [N/m]
            this.params.k_p_ub	= 75.0e3; 	% Stiffness upper bound [N/m]
            this.params.p_lb   	= 0.00;   	% Pretension position lower bound [m]
            this.params.p_ub   	= 0.20;   	% Pretension position upper bound [m]
            
            % Define default parameters
            this.params.r_0     = 0.07;     % Default pulley radius [m]
            this.params.k_p_0   = 15.0e3;   % Default ESB stiffness [N/m]
            this.params.p_0     = 0.10;     % Default pretension [m] - required >> 0 so that there is tension
            this.params.r_0_array = [];     % Array of chosen pulley radii [m] (for independent chosen params per actuator)
            
            % Tolerances
            this.params.funTol	= 1e-8;   	% Function tolerance
            this.params.xTol	= 1e-8;   	% Solution tolerance
            this.params.MaxIter = 1000;     % Max iterations
            this.params.MaxFunEvals = 8000; % Max function evaluations
            
            % Integration resolution
            this.params.res     = 0.05;     % Resolution [rad]
            
            % Set default actuation topology
            % Just -1, 0, +1 values here, to indicate connections and
            % directions
            
            % Monoarticulated configuration
            T = [   0, 0, 0,    -1,     0,      0; ...  % ACA 1
                    0, 0, 0,    0,      1,      0; ...  % ACA 2
                    0, 0, 0,    0,      0,      0   ];  % ACA 3
               %x1 %y1 %theta1  %q1     %q2     %q3
            
            % Biarticulated configuration
%             T = [   0, 0, 0,    -1,     -1,     0; ...  % ACA 1
%                     0, 0, 0,    0,      1,      0; ...  % ACA 2
%                     0, 0, 0,    0,      0,      0   ];  % ACA 3
               %x1 %y1 %theta1  %q1     %q2     %q3
                    
            lambda = [0 0 0 1 1 1]; % Weighting vector lambda
            this.setTopology(T, lambda);
            
            % Create optimization range (q_range)
            this.createOptRange();
            
            % Status
            disp('Initialized Leg_3DoF_ACA_optimizer with default parameters.');
        end
        
        %__________________________________________________________________
        % Create optimization range (q_range)
        function [] = createOptRange(this)
            % Make joint space for q_range
            
            % Preallocate
            q_range     = [];
            x_hip_all	= [];
            y_hip_all	= [];
            G_range     = [];
            
            % Create a q_range based on linear joint space above stance foot
            l2          = this.model.leg.params.l2; % l2 (shank) shorthand
            l3          = this.model.leg.params.l3; % l3 (thigh) shorthand
            r_limits	= this.params.r_limits;
            beta_limits	= this.params.beta_limits;
            
            % Get joint limits for q1,q2 (we consider q3 dependent)
            q_1_limits = this.model.leg.params.q_limits(1,:);
            q_2_limits = this.model.leg.params.q_limits(2,:);
            
            % Get number of components for each joint
            N = round(abs(max(q_1_limits)-min(q_1_limits)) / this.params.res); % Number of q_1 components
            M = round(abs(max(q_2_limits)-min(q_2_limits)) / this.params.res); % Number of q_2 components

            % Loop over all configurations
            for i=1:N
                for j=1:M
                    % Get q for this point in the grid
                    q1 = this.model.leg.params.q_limits(1,1) + i*this.params.res;
                    q2 = this.model.leg.params.q_limits(2,1) + j*this.params.res;
                    q3 = -(q1+q2);
                    q = [0; 0; 0; q1; q2; q3];

                    % Calculate Cartesian coordinates of hip
                    [x_hip, y_hip, ~] = this.model.leg.calc_fwdKin_named(q, 'hip');
                    
                    % Calculate hip x-position w.r.t. the ankle joint for inverse
                    % kinematics
                    x_hip_ankle = x_hip + this.model.leg.params.r1;

                    % Calculate polar coordinates from Cartesian
                    beta        = atan2(-x_hip_ankle, y_hip);
                    r           = sqrt(x_hip_ankle^2 + y_hip^2);
                    
                    % Check the CoP for this configuration if desired
                    if (this.params.checkCoP)
                        % Obtain the static CoP from the leg model.
                        [ ~, CoP_static ] = this.model.leg.calc_GRF_CoP_static(q);

                        % CoP check
                        if (    CoP_static < -this.model.leg.params.r_heel  || ...
                                CoP_static > this.model.leg.params.r_toe  	)
                            continue;
                        end
                    end

                    % Polar coordinates check if desired
                    if (    this.params.checkPolar && ...
                            (   beta < beta_limits(1) || beta > beta_limits(2)  || ...
                                r < r_limits(1) || r > r_limits(2)              )           )
                        continue;
                    end
                    
                    % Checks passed, add this configuration
                    q_range(:,end+1) = q;
                    x_hip_all(end+1) = x_hip_ankle;
                    y_hip_all(end+1) = y_hip;
                    G_range(:,end+1) = this.model.leg.calc_G(q);
                end
            end

%             % Create a q_range based on the reference
%             q_range = [];
%             ref         = this.model.ref;
%             period      = (2*pi)/ref.omega;
%             for i=0:this.params.res:period
%                 % Get q and add to range
%                 q = this.model.q_ref(i);
%                 q_range(:,end+1) = q;
%                 
%                 % Calculate Cartesian coordinates of hip
%                 [x_hip_all(end+1), y_hip_all(end+1), ~] = ...
%                     this.model.leg.calc_fwdKin_named(q, 'hip');
%             end

            % Set q_range
            this.params.q_range = q_range;
            
            % Limited q_range for debugging
            %posture1 = [0; 0; 0; -0.3; 0.6; -0.3];
            %posture2 = [0; 0; 0; -1.0; 1.0; 0.0];
            %posture3 = [0; 0; 0; 0.0; 0.0; 0.0];
            %this.params.q_range = [posture1, posture2, posture3];
            
            % Ask whether to do plots
            if (~confirm('Show plots of optimisation workspace? [Y/n]', 1))
                return;
            end
            
            % Ask about paperMode
            paperMode = confirm('Create plots in paper mode? [y/N]', 0);
            
            % Plot
            h = figure;
            if (paperMode)
                resizeFig(h, 500, 190);
            else
                resizeFig(h, 600, 400);
            end
            
            % q1,q2 plot and joint limits
            q_limits = this.model.leg.params.q_limits;
            q1_plot_lim = q_limits(1,:) + [-0.1, 0.1];
            q2_plot_lim = q_limits(2,:) + [-0.1, 0.1];
            
            subplot(1,2,1); grid on; hold on;
            plot(q_range(4,:), q_range(5,:), '.');
            plot([q_limits(1,1), q_limits(1,1)], q2_plot_lim, '--', 'Color', 'black');
            plot([q_limits(1,2), q_limits(1,2)], q2_plot_lim, '--', 'Color', 'black');
            plot(q1_plot_lim, [q_limits(2,1), q_limits(2,1)], '--', 'Color', 'black');
            plot(q1_plot_lim, [q_limits(2,2), q_limits(2,2)], '--', 'Color', 'black');
            
            xlim(q1_plot_lim); ylim(q2_plot_lim);
            paperModeAxisLabels(paperMode, {'$q_1$ [rad]', '$q_2$ [rad]'}, {'q_1', 'q_2'});
            paperModeTitle(1, paperMode, 'Joint space');
            
            % x,y plot
            subplot(1,2,2); grid on; hold on;
            scatter(x_hip_all, y_hip_all, '.');
            if (paperMode)
                y_max = this.model.leg.params.l2 + this.model.leg.params.l3 + 0.1;
                xlim([-0.25 0.30]); ylim([0 y_max]);
            else
                xlim([-0.3 0.4]); ylim([0 1.0]);
            end
            paperModeAxisLabels(paperMode, {'$x$ [m] (w.r.t. ankle)', '$y$ [m]'}, {'x (w.r.t. ankle)', 'y'});
            paperModeTitle(1, paperMode, 'Cartesian space (hip)');
            
            % Draw polar angle lines in plot
            % Only draw these when paperMode is off and checkPolar is on
            if (~paperMode && this.params.checkPolar)
                r_line = l2+l3;
                x_line = r_line * cos(beta_limits(1)+pi/2);
                y_line = r_line * sin(beta_limits(1)+pi/2);
                plot([0 x_line], [0 y_line], '--', 'Color', 'black');
                x_line = r_line * cos(beta_limits(2)+pi/2);
                y_line = r_line * sin(beta_limits(2)+pi/2);
                plot([0 x_line], [0 y_line], '--', 'Color', 'black');
                text(-0.2, 1.2*(l2+l3), ['beta=' num2str(beta_limits(1)) '...' num2str(beta_limits(2))]);
                text(-0.2, 1.1*(l2+l3), ['r=' num2str(r_limits(1)) '...' num2str(r_limits(2))]);
            end
            
            % Draw support polygon
            % Only draw these when paperMode is off
            if (~paperMode)
                x_line = -this.model.leg.params.r_heel;
                plot([x_line x_line], [0 1.05*(l2+l3)], '--', 'Color', 'black');
                x_line = this.model.leg.params.r_toe;
                plot([x_line x_line], [0 1.05*(l2+l3)], '--', 'Color', 'black');
            end
            
            % Gravitational torque landscape
            h = figure; grid on; hold on;
            if (paperMode)
                resizeFig(h, 400, 320);
            else
                resizeFig(h, 600, 580);
            end
            
            h1 = scatter3(q_range(4,:), q_range(5,:), G_range(4,:), '.r');
            h2 = scatter3(q_range(4,:), q_range(5,:), G_range(5,:), '.g');
            h3 = scatter3(q_range(4,:), q_range(5,:), G_range(6,:), '.b');
            xlim([min(q_range(4,:)) max(q_range(4,:))]);
            ylim([min(q_range(5,:)) max(q_range(5,:))]);
            zlim('auto');
            paperModeLegend(    paperMode, ...
                                {   ['$q_1$ (' num2str(min(G_range(4,:)), '%2.0f') '...' num2str(max(G_range(4,:)), '%2.0f') ' Nm)'], ...
                                    ['$q_2$ (' num2str(min(G_range(5,:)), '%2.0f') '...' num2str(max(G_range(5,:)), '%2.0f') ' Nm)'], ...
                                    ['$q_3$ (' num2str(min(G_range(6,:)), '%2.0f') '...' num2str(max(G_range(6,:)), '%2.0f') ' Nm)']  }, ...
                                {   ['R: Joint 1 (min/max: ' num2str(min(G_range(4,:)), '%2.0f') '...' num2str(max(G_range(4,:)), '%2.0f') ' Nm)'], ...
                                    ['G: Joint 2 (min/max: ' num2str(min(G_range(5,:)), '%2.0f') '...' num2str(max(G_range(5,:)), '%2.0f') ' Nm)'], ...
                                    ['B: Joint 3 (min/max: ' num2str(min(G_range(6,:)), '%2.0f') '...' num2str(max(G_range(6,:)), '%2.0f') ' Nm)']  }, ...
                                [h1,h2,h3] ...
            );
            paperModeTitle(~paperMode, paperMode, 'Gravitational torques on joints');
            paperModeAxisLabels(paperMode, {'$q_1$ (ankle)', '$q_2$ (knee)'}, {'Joint 1 (ankle)', 'Joint 2 (knee)'});
            if (paperMode)
                zlabel('Torque [N m]', 'Interpreter', 'LaTeX');
            else
                zlabel('Torque [N m]');
            end
            set(gca,'Box','on');
            set(gca,'PlotBoxAspectRatioMode','manual');
            view(45,20);
            
            % Draw plots            
            drawnow;
        end
        
        %__________________________________________________________________
        % Set topology type (mono / bi / no ESB / etc)
        % Matrix T should be NxN, where N = leg.N, to imply a fully
        % articulated system. Matrix T contains +1's and -1's to signify
        % the the actuation topology in terms of connectivity.
        function [] = setTopology(this, T, lambda)
            % Set topology matrix
            N = this.model.leg.N; % To not count floating base states
            if (size(T,2) ~= N/2)
                error(['Topology matrix has invalid size: ' num2str(size(T))]);
            end
            this.params.T = T;
            
            % Error vector weights (lambda)
            if (length(lambda) ~= N/2)
                error(['Weighting vector lambda has invalid size: ' num2str(length(lambda))]);
            end
            this.params.lambda = lambda;
            
            % Loop over the rows of T, to see which joints are
            % monoarticulated, biarticulated, N-articulated, or no ESB
            this.params.articulation = {};
            for i=1:size(T,1)
                art = length(nonzeros(T(i,:))); % Find nonzero elements in row

                % We only support no ESB, mono-, bi- and tri-articulation
                % for now
                switch (art)
                    case 0
                        % No ESB
                        this.params.articulation{i} = 'noESB';
                    case 1
                        % Monoarticulation
                        this.params.articulation{i} = 'mono';
                    case 2
                        % Biarticulation
                        this.params.articulation{i} = 'bi';
                    case 3
                        % Triarticulation
                        this.params.articulation{i} = 'tri';
                    otherwise
                        error(['Unsupported articulation: ' num2str(T)]);
                end
            end
            
            % Status
            disp('Set topology matrix T:');
            T %#ok<*NOPRT>
            disp('with articulation:');
            disp(this.params.articulation);
        end
        
        %__________________________________________________________________
        % Run optimization
        function [ psi, T, actuatorParams ] = run(this)
         	% Calculate the optimal values of the pulley radii, pretension positions
         	% and ESB stiffnesses for the 2-DoF leg s.t. the ESB torques match the
        	% gravitational torque as close as possible.
            
            % Set empty struct for results
            disp('Clearing any previous results..');
            this.results = struct;
            
            % Create optimization range (q_range)
            disp('Regenerating optimisation workspace in case parameters have been changed..');
            this.createOptRange();
            
            % Nonlinear optimization
            disp('Building initial parameter vector x0..');
            
            % Build psi elements for each actuator, as well as lower and
            % upper bounds
            
            % Parameters
            r_0         = this.params.r_0;
            k_p_0       = this.params.k_p_0;
            p_0         = this.params.p_0;
            r_lb        = this.params.r_lb;
            k_p_lb      = this.params.k_p_lb;
            p_lb        = this.params.p_lb;
            r_ub        = this.params.r_ub;
            k_p_ub      = this.params.k_p_ub;
            p_ub        = this.params.p_ub;
            r_0_array   = this.params.r_0_array;
            
            % Check whether r_0_array has contents and is equal in size to
            % the number of actuators.
            chosenParamsArray = 0;
            if (length(r_0_array) > 1)
                if (length(r_0_array) ~= size(this.params.T,1))
                    this.params.T
                    r_0_array
                    error('Invalid size of r_0_array!');
                end
                chosenParamsArray = 1;
            end
            
            % Build psi, psi_lb, psi_ub
            psi     = [];
            psi_lb  = [];
            psi_ub  = [];
            for i=1:size(this.params.T,1)
                % If we have an array for the chosen parameters, set it
                if (chosenParamsArray == 1)
                    r_0 = r_0_array(i);
                end
                
                if (strcmp(this.params.articulation{i}, 'noESB'))
                    % Do nothing, no ESB parameters to optimize for noESB
                    
                elseif (strcmp(this.params.articulation{i}, 'mono'))
                    % psi
                    psi_1	= r_0 * k_p_0 * p_0;
                    psi_2	= r_0^2 * k_p_0;
                    psi     = [psi; psi_1; psi_2];
                    
                    % lb
                    psi_1_lb    = r_ub * k_p_ub * p_lb; % p can be negative, i.e. r_ub, k_p_ub
                    psi_2_lb    = r_lb^2 * k_p_lb;
                    psi_lb      = [psi_lb; psi_1_lb; psi_2_lb];
                    
                    % ub
                    psi_1_ub    = r_ub * k_p_ub * p_ub;
                    psi_2_ub    = r_ub^2 * k_p_ub;
                    psi_ub      = [psi_ub; psi_1_ub; psi_2_ub];
                    
                elseif (strcmp(this.params.articulation{i}, 'bi'))
                    % psi
                    psi_1   = sqrt(k_p_0) * r_0;
                    psi_2   = sqrt(k_p_0) * r_0;
                    psi_3   = sqrt(k_p_0) * p_0;
                    psi     = [psi; psi_1; psi_2; psi_3];
                    
                    % lb
                    psi_1_lb    = sqrt(k_p_lb) * r_lb;
                    psi_2_lb    = sqrt(k_p_lb) * r_lb;
                    psi_3_lb    = sqrt(k_p_ub) * p_lb; % p can be negative, i.e. k_p_ub
                    psi_lb      = [psi_lb; psi_1_lb; psi_2_lb; psi_3_lb];
                    
                    % ub
                    psi_1_ub    = sqrt(k_p_ub) * r_ub;
                    psi_2_ub    = sqrt(k_p_ub) * r_ub;
                    psi_3_ub    = sqrt(k_p_ub) * p_ub;
                    psi_ub      = [psi_ub; psi_1_ub; psi_2_ub; psi_3_ub];
                    
                elseif (strcmp(this.params.articulation{i}, 'tri'))
                    % TODO
                    
                else
                    error('articulation value error');
                end
            end
            
            % Build x0, x_lb, x_ub
            x0      = psi;
            x_lb	= psi_lb;
            x_ub	= psi_ub;
            printVector(x0, 'x0');
            printVector(x_lb, 'x_lb');
            printVector(x_ub, 'x_ub');
            
            % Optimisation options
            TypicalX = x0; % Typical X values (used for gradient scaling)
            opt = optimoptions( @fmincon, ...
                                'Display',      'final-detailed', ...
                                'TolFun',       this.params.funTol, ...
                                'TolX',         this.params.xTol, ...
                                'MaxIter',      this.params.MaxIter, ...
                                'MaxFunEvals',	this.params.MaxFunEvals, ...
                                'TypicalX',     TypicalX    );
                            
            % Constrained nonlinear optimization
            disp('Starting CONSTRAINED nonlinear optimisation..');
            [x, fval, exitflag, output, grad] = fmincon( ...
                @(a1) this.calc_error(a1), ...
                x0, ...
                zeros(length(x0)), ...
                zeros(size(x0)), ...
                [], [], x_lb, x_ub, ...
                [], opt); %#ok<*ASGLU>

            % Show result
            disp(['Final value of error function is ' num2str(fval) '.']);
            disp('Resulting optimal vector x is:');
            printVector(x, 'x');
            
            % Value of x now contains the optimal psi values
            psi = x;
            clear x;
            
            % Show a warning if one of the components of psi is at its lower
            % or upper bound
            for i=1:length(psi)
                if (abs(psi(i) - psi_lb(i)) < 1e-6*abs(psi_lb(i)))
                    warning(['psi(' num2str(i) ') is at its lower bound.']); 
                elseif (abs(psi(i) - psi_ub(i)) < 1e-6*abs(psi_ub(i)))
                    warning(['psi(' num2str(i) ') is at its upper bound.']);
                end
            end
            
            % Calculate resulting parameters for each actuator
            clear actuatorParams;   % To store the resulting actuator params
            it = 1; % Iterator in psi to keep track of which params belong to which actuator
            for i=1:size(this.params.T,1)
                % Prepare params struct for this actuator
                aParams         = struct;
                aParams.type	= this.params.articulation{i};
                
                % If we have an array for the chosen parameters, set it
                if (chosenParamsArray == 1)
                    r_0 = r_0_array(i);
                end
                
                if (strcmp(this.params.articulation{i}, 'noESB'))
                    % No ESB parameters to optimize for noESB
                    disp(['Actuator ' num2str(i) ': No ESB']);
                    
                    % Set some default values
                    aParams.k_p     = 0;
                    aParams.r       = 0;
                    aParams.p       = 0;
                    
                elseif (strcmp(this.params.articulation{i}, 'mono'))
                    % Get components from psi
                    psi_1	= psi(it);
                    psi_2	= psi(it+1);
                    
                    % Assume the pulley radius r chosen/given
                    r       = r_0;
                    k_p     = psi_2 / r^2;
                    p       = r * (psi_1 / psi_2);
                    
                    % Check if the actuator's parameters are within bounds.
                    % If not, show a warning.
                    if (k_p < this.params.k_p_lb || k_p > this.params.k_p_ub)
                        disp(['WARN: Actuator ' num2str(i) ' parameter k_p = ' num2str(k_p) ' violates bounds [' num2str(this.params.k_p_lb) ', ' num2str(this.params.k_p_ub) '] for chosen parm r = ' num2str(r) ' !']);
                        if (confirm('Do you want to saturate this parameter to its bound [y/N]?', 0))
                            k_p = min(this.params.k_p_ub, max(this.params.k_p_lb, k_p)); % Clever way to get a saturation function
                        end
                        disp('-------------');
                    end
                    if (p < this.params.p_lb || p > this.params.p_ub)
                        disp(['WARN: Actuator ' num2str(i) ' parameter p = ' num2str(p) ' violates bounds [' num2str(this.params.p_lb) ', ' num2str(this.params.p_ub) '] for chosen parm r = ' num2str(r) ' !']);
                        if (confirm('Do you want to saturate this parameter to its bound [y/N]?', 0))
                            p = min(this.params.p_ub, max(this.params.p_lb, p)); % Clever way to get a saturation function
                        end
                        disp('-------------');
                    end
                    
                    % Rebuild psi components in case we saturated above
                    psi_1         	= r * k_p * p;
                    psi_2           = r^2 * k_p;
                    psi(it)         = psi_1;
                    psi(it+1)       = psi_2;
                    
                    % Set values into actuator parameters struct
                    aParams.r       = r;
                    aParams.k_p     = k_p;
                    aParams.p       = p;
                    
                    % Display result
                    disp(['Actuator ' num2str(i) ': Monoarticulated with ' ...
                            '(r, k_p, p) = (' num2str(r) ', ' num2str(k_p) ....
                            ', ' num2str(p) ')']);
                        
                    % Increment iterator
                    it = it+2;
                    
                elseif (strcmp(this.params.articulation{i}, 'bi'))
                    % Get components from psi
                    psi_1	= psi(it);
                    psi_2	= psi(it+1);
                    psi_3	= psi(it+2);
                    
                    % Assume the end pulley radius r_bi_1 given
                    r_bi_1  = r_0;
                    k_p    	= (psi_1 / r_bi_1)^2;
                    r_bi_2 	= psi_2 / sqrt(k_p);
                    p     	= psi_3 / sqrt(k_p);
                    
                    % Check if the actuator's parameters are within bounds.
                    % If not, show a warning.
                    if (k_p < this.params.k_p_lb || k_p > this.params.k_p_ub)
                        disp(['WARN: Actuator ' num2str(i) ' parameter k_p = ' num2str(k_p) ' violates bounds [' num2str(this.params.k_p_lb) ', ' num2str(this.params.k_p_ub) '] for chosen parm r_bi_1 = ' num2str(r_bi_1) ' !']);
                        if (confirm('Do you want to saturate this parameter to its bound [y/N]?', 0))
                            k_p = min(this.params.k_p_ub, max(this.params.k_p_lb, k_p)); % Clever way to get a saturation function
                        end
                        disp('-------------');
                    end
                    if (r_bi_2 < this.params.r_lb || r_bi_2 > this.params.r_ub)
                        disp(['WARN: Actuator ' num2str(i) ' parameter r_bi_2 = ' num2str(r_bi_2) ' violates bounds [' num2str(this.params.r_lb) ', ' num2str(this.params.r_ub) '] for chosen parm r_bi_1 = ' num2str(r_bi_1) ' !']);
                        if (confirm('Do you want to saturate this parameter to its bound [y/N]?', 0))
                            r_bi_2 = min(this.params.r_ub, max(this.params.r_lb, r_bi_2)); % Clever way to get a saturation function
                        end
                        disp('-------------');
                    end
                    if (p < this.params.p_lb || p > this.params.p_ub)
                        disp(['WARN: Actuator ' num2str(i) ' parameter p = ' num2str(p) ' violates bounds [' num2str(this.params.p_lb) ', ' num2str(this.params.p_ub) '] for chosen parm r_bi_1 = ' num2str(r_bi_1) ' !']);
                        if (confirm('Do you want to saturate this parameter to its bound [y/N]?', 0))
                            p = min(this.params.p_ub, max(this.params.p_lb, p)); % Clever way to get a saturation function
                        end
                        disp('-------------');
                    end
                    
                    % Rebuild psi components in case we saturated above
                    psi_1           = sqrt(k_p) * r_bi_1;
                    psi_2           = sqrt(k_p) * r_bi_2;
                    psi_3           = sqrt(k_p) * p;
                    psi(it)         = psi_1;
                    psi(it+1)       = psi_2;
                    psi(it+2)       = psi_3;
                    
                    % Set values into actuator parameters struct
                    aParams.r_bi_1    	= r_bi_1;
                    aParams.k_p         = k_p;
                    aParams.r_bi_2   	= r_bi_2;
                    aParams.p           = p;
                    
                    % Display result
                    disp(['Actuator ' num2str(i) ': Biarticulated with ' ...
                            '(r_bi_1, r_bi_2, k_p, p) = (' num2str(r_bi_1) ...
                            ', ' num2str(r_bi_2) ', ' num2str(k_p) ...
                            ', ' num2str(p) ')']);
                        
                    % Increment iterator
                    it = it+3;
                    
                elseif (strcmp(this.params.articulation{i}, 'tri'))
                    disp(['Actuator ' num2str(i) ': Triarticulation']);
                    % TODO
                    
                else
                    error('articulation value error');
                end
                
                % Add actuator parameters to output
                actuatorParams{i} = aParams; %#ok<*AGROW>
                
                disp('-------------');
            end
            
            % Calculate topology matrix from actuatorParams
            T = this.topologyFromActuatorParams(actuatorParams);            
            
            % Set results into results struct
            this.results.actuatorParams     = actuatorParams;
            this.results.T                  = T;
            this.results.psi                = psi;
            
            % Offer to save results
            if (confirm('Do you want to save the results to optimizationResults.mat [Y/n]?', 1))
                save('optimizationResults.mat', 'psi', 'T', 'actuatorParams');
                disp('Data saved to optimizationResults.mat.');
            end
            
            % Offer to plot results
            if (confirm('Would you like to plot the results? [Y/n]', 1))
                this.plot();
            end
        end
        
        %__________________________________________________________________
        % Plot optimisation results
        function [ ] = plot(this)
            % The plot code below uses psi formulation, and thus the
            % topology matrix T from this.params.T (which has only -1, 0,
            % and +1 values), instead of the one from the results, which
            % includes pulley radii.
            
            % Get psi from results struct
            psi = this.results.psi;
            
            % Plot calculations
            
            % Calculate surface plots for the torques
            % Range of motion
            d2r = (pi/180); % deg2rad
            q_1_r_surf 	= [-60*d2r	: 0.1	: 60*d2r    ]; %#ok<*NBRAK>
            q_2_r_surf 	= [0        : 0.1	: 180*d2r   ];

            % Create grids for plotting
            [q_1_grid, q_2_grid]	= meshgrid(q_1_r_surf, q_2_r_surf);
            G_grid                  = zeros([size(q_1_grid),this.model.leg.N/2]);
            tau_p_grid              = zeros([size(q_1_grid),size(this.params.T,1),this.model.leg.N/2]);
            % See notes on dimensions below

            % Calculate tau_p value grid and G value grid
            % tau_p_grid(i,j,k,l): where i,j denote q1,q2 configuration, k
            % denotes the actuator, and l denotes the leg DoF (i.e. q1=4,
            % q2=5, and q3=6).
            % G_grid(i,j,l): where i,j denote q1,q2 configuration, and l
            % denotes the joint
            for i=1:size(q_1_grid,1)
                for j=1:size(q_1_grid,2)
                    % Get pose
                    q = [0; 0; 0; q_1_grid(i,j); q_2_grid(i,j); -q_1_grid(i,j)-q_2_grid(i,j)];
                    
                    % Get gravitational torques for this pose
                    G_grid(i,j,:) = reshape(this.model.leg.calc_G(q), [1 1 length(q)]);

                    % Calculate the torque generated by each actuator
                    it = 1;
                    for k=1:size(this.params.T,1)
                        % Find the type of actuator to calculate its torque
                        if (strcmp(this.params.articulation{k}, 'noESB'))
                            % Do nothing, no ESB parameters to optimize for noESB

                        elseif (strcmp(this.params.articulation{k},'mono'))
                            psi_1 = psi(it);
                            psi_2 = psi(it+1);
                            tau_p_grid(i,j,k,:) = this.calc_tau_p_psi(q, k, psi_1, psi_2);
                            it = it+2;

                        elseif (strcmp(this.params.articulation{k}, 'bi'))
                            psi_1 = psi(it);
                            psi_2 = psi(it+1);
                            psi_3 = psi(it+2);
                            tau_p_grid(i,j,k,:) = this.calc_tau_p_biarticulated_psi(q, k, psi_1, psi_2, psi_3);
                            it = it+3;

                        elseif (strcmp(this.params.articulation{k}, 'tri'))
                            % TODO

                        else
                            error('articulation value error');
                        end
                    end
                end
            end
            
            % Calculate the net ESB joint torques by summing the
            % contributions from all actuators
            tau_p_1_grid = zeros([size(q_1_grid)]);
            tau_p_2_grid = zeros([size(q_1_grid)]);
            for k=1:size(tau_p_grid,3)
                tau_p_1_grid = tau_p_1_grid + tau_p_grid(:,:,k,4);
                tau_p_2_grid = tau_p_2_grid + tau_p_grid(:,:,k,5);
            end

            % Get data points for desired workspace
            q_1_curve = this.params.q_range(4,:);
            q_2_curve = this.params.q_range(5,:);
            z_curve = ones(size(q_2_curve));

            % Actual plots
            
            % Ask about paperMode
            paperMode = confirm('Create plots in paper mode? [y/N]', 0);
            
            % Find correct mode, defaulting to mono and finding an
            % occorrance of 'bi'
            mode = 'mono';
            for i=1:size(this.params.T,1)
                if (strcmp(this.params.articulation{i}, 'bi'))
                   mode = 'bi';
                   break;
                end
            end

            if (strcmp(mode, 'mono'))
                % G_1: Gravitational torque on q_1
                h = figure('renderer','zbuffer'); clf; hold on;
                if (paperMode)
                    resizeFig(h, 400, 320);
                else
                    resizeFig(h, 600, 580);
                end
                surf(q_1_grid, q_2_grid, G_grid(:,:,4));
                surf(q_1_grid, q_2_grid, -tau_p_1_grid, 'FaceColor', [1 0 0]);
                z_curve_min = min([min(G_grid(:,:,4)) min(-tau_p_1_grid)]);
                scatter3(q_1_curve, q_2_curve, z_curve_min*z_curve, 'filled');
                paperModeAxisLabels(paperMode, {'$q_1$ [rad]', '$q_2$ [rad]'}, {'q_1', 'q_2'});
                if (paperMode)
                    zlabel('Torque [N m]', 'Interpreter', 'LaTeX');
                else
                    zlabel('Torque [N m]');
                end
                paperModeLegend(    paperMode, ...
                                    {'$G_1$', '$-\tau_{p,1}$'}, ...
                                    {'G_1', 'Monoart. torque -tau_{p,1}'} ...
                );
                paperModeTitle(~paperMode, paperMode, 'Gravitational torque on $q_1$', 'G_1: Gravitational torque on q_1');
                xlim([min(q_1_r_surf) max(q_1_r_surf)]);
                ylim([min(q_2_r_surf) max(q_2_r_surf)]);
                zlim('auto'); axis tight;
                set(gca,'Box','on');
                set(gca,'PlotBoxAspectRatioMode','manual');
                view(45,20);
                %zlim([-50 50]);

                % G_2: Gravitational torque on q2
                h = figure('renderer','zbuffer'); clf; hold on;
                if (paperMode)
                    resizeFig(h, 400, 320);
                else
                    resizeFig(h, 600, 580);
                end
                surf(q_1_grid, q_2_grid, G_grid(:,:,5));
                surf(q_1_grid, q_2_grid, -tau_p_2_grid, 'FaceColor', [1 0 0]);
                z_curve_min = min([min(G_grid(:,:,5)) min(-tau_p_2_grid)]);
                scatter3(q_1_curve, q_2_curve, z_curve_min*z_curve, 'filled');
                paperModeAxisLabels(paperMode, {'$q_1$ [rad]', '$q_2$ [rad]'}, {'q_1', 'q_2'});
                if (paperMode)
                    zlabel('Torque [N m]', 'Interpreter', 'LaTeX');
                else
                    zlabel('Torque [N m]');
                end
                paperModeLegend(    paperMode, ...
                                    {'$G_2$', '$-\tau_{p,2}$'}, ...
                                    {'G_2', 'Monoart. torque -tau_{p,2}'} ...
                );
                paperModeTitle(~paperMode, paperMode, 'Gravitational torque on $q_2$', 'G_2: Gravitational torque on q_2');
                xlim([min(q_1_r_surf) max(q_1_r_surf)]);
                ylim([min(q_2_r_surf) max(q_2_r_surf)]);
                zlim('auto'); axis tight;
                set(gca,'Box','on');
                set(gca,'PlotBoxAspectRatioMode','manual');
                view(45,20);
                
            elseif (strcmp(mode, 'bi'))
                % G_1: Gravitational torque on q_1
                h = figure('renderer','zbuffer'); clf; hold on;
                if (paperMode)
                    resizeFig(h, 400, 320);
                else
                    resizeFig(h, 600, 580);
                end
                surf(q_1_grid, q_2_grid, G_grid(:,:,4));
                surf(q_1_grid, q_2_grid, -tau_p_1_grid, 'FaceColor', [1 0 0]);
                z_curve_min = min([min(G_grid(:,:,4)) min(-tau_p_1_grid)]);
                scatter3(q_1_curve, q_2_curve, z_curve_min*z_curve, 'filled');
                paperModeAxisLabels(paperMode, {'$q_1$ [rad]', '$q_2$ [rad]'}, {'q_1', 'q_2'});
                if (paperMode)
                    zlabel('Torque [N m]', 'Interpreter', 'LaTeX');
                else
                    zlabel('Torque [N m]');
                end
                paperModeLegend(    paperMode, ...
                                    {'$G_1$', '$-\tau_{p,1}$'}, ...
                                    {'G_1', 'Biart. torque -tau_{p,1}'} ...
                );
                paperModeTitle(~paperMode, paperMode, 'Gravitational torque on $q_1$', 'G_1: Gravitational torque on q_1');
                xlim([min(q_1_r_surf) max(q_1_r_surf)]);
                ylim([min(q_2_r_surf) max(q_2_r_surf)]);
                zlim('auto'); axis tight;
                set(gca,'Box','on');
                set(gca,'PlotBoxAspectRatioMode','manual');
                view(45,20);
                %zlim([-50 50]);
                
                % G_2: Gravitational torque on q2
                h = figure('renderer','zbuffer'); clf; hold on;
                if (paperMode)
                    resizeFig(h, 400, 320);
                else
                    resizeFig(h, 600, 580);
                end
                surf(q_1_grid, q_2_grid, G_grid(:,:,5));
                if (paperMode)
                    surf(q_1_grid, q_2_grid, -tau_p_2_grid, 'FaceColor', [1 0 0]);
                    z_curve_min = min([min(G_grid(:,:,5)) min(-tau_p_2_grid)]);
                else
                    h = surf(q_1_grid, q_2_grid, -tau_p_grid(:,:,2,5), 'FaceColor', [1 1 0]);
                    alpha(h, 0.5);
                    h = surf(q_1_grid, q_2_grid, -tau_p_grid(:,:,1,5), 'FaceColor', [0 1 1]);
                    alpha(h, 0.5);
                    h = surf(q_1_grid, q_2_grid, -tau_p_2_grid, 'FaceColor', [1 0 0]);
                    alpha(h, 0.5);
                    z_curve_min = min([min(G_grid(:,:,5)) min(-tau_p_grid(:,:,2,5)) min(-tau_p_grid(:,:,1,5)) min(-tau_p_2_grid)]);
                end
                scatter3(q_1_curve, q_2_curve, z_curve_min*z_curve, 'filled');
                paperModeAxisLabels(paperMode, {'$q_1$ [rad]', '$q_2$ [rad]'}, {'q_1', 'q_2'});
                if (paperMode)
                    zlabel('Torque [N m]', 'Interpreter', 'LaTeX');
                else
                    zlabel('Torque [N m]');
                end
                paperModeLegend(    paperMode, ...
                                    {'$G_2$', '$-\tau_{p,2}$'}, ...
                                    {'G_2', 'Monoart. torque -tau_{p,mono}', 'Biarticulated torque -tau_{p,bi}', 'Net torque -tau_{p,2}'} ...
                );
                paperModeTitle(~paperMode, paperMode, 'Gravitational torque on $q_2$', 'G_2: Gravitational torque on q_2');
                xlim([min(q_1_r_surf) max(q_1_r_surf)]);
                ylim([min(q_2_r_surf) max(q_2_r_surf)]);
                zlim('auto'); axis tight;
                set(gca,'Box','on');
                set(gca,'PlotBoxAspectRatioMode','manual');
                view(45,20);
                
            else
                error('Invalid mode for plotting');
            end
            
%             % Calculate torques along entire desired workspace and plot in 2D
%             for i=1:size(this.params.q_range,2)
%                 [~, tau_p_net(:,i)] = this.calc_tau_p(this.params.q_range(:,i), psi);
%             end
%
%             % Get gravitational torques for the poses
%             G_curve = this.model.leg.calc_G(this.params.q_range);
% 
%             % Torques on q_1
%             figure('renderer','zbuffer'); clf; hold on;
%             set(gcf,'Position', [300 250 500 400]);
%             plot(G_curve(1,:), '.', 'Color', [0 0.7 0]);
%             plot(-tau_p_net(1,:), '.r');
%             xlabel('Sample'); ylabel('Torque [N m]');
%             legend('G_1', '-tau_{p,1}');
%             title('Gravitational vs ESB torque on q_1 along curve');
% 
%             % Torques on q_2
%             figure('renderer','zbuffer'); clf; hold on;
%             set(gcf,'Position', [900 250 500 400]);
%             plot(G_curve(2,:), '.', 'Color', [0 0.7 0]);
%             plot(-tau_p_net(2,:), '.r');
%             xlabel('Sample'); ylabel('Torque [N m]');
%             legend('G_2', '-tau_{p,2}');
%             title('Gravitational vs ESB torque on q_2 along curve');
        end
        
        %__________________________________________________________________
        % Perform integral to get the error vector for this parameter vector x.
        function [ E ] = calc_error(this, x)
            % Status
            %disp('Calculating error vector with parameter vector x:');
            %printVector(x, 'x');

            % Define initial errors
            E = zeros(1,size(this.params.T,2));

            % Perform 'integral'
            for i=1:size(this.params.q_range,2)
                % Get pose q
                q = this.params.q_range(:,i);

                % Calculate error vector and increment
                e = this.calc_error_pose(q, x);
                E = E + this.params.res * e.^2;
            end

            % Weigh E
            E = (this.params.lambda / norm(this.params.lambda)) .* E;

            % Result
            %disp(['Total error vector E (norm ' num2str(norm(E)) '):']);
            %printVector(E, 'E');
            %disp('-----');

            % Return norm of error as fmincon expects a scalar
            E = norm(E);
            
            % Debug
            %pause;
        end
        
        %__________________________________________________________________
        % Calculates the error vector between the ESB torque and gravitational
        % torque for a given pose.
        function [ e ] = calc_error_pose(this, q, x)
            % Calculate gravitational torque vector for this pose
            G = this.model.leg.calc_G(q)';
            
            % Calculate the ESB torques
            [~, tau_p_net] = this.calc_tau_p(q, x);

            % Calculate error vector
            e = G + tau_p_net;
            
            % Debug
            %disp(['For q = [' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) '], ' ...
            %    'G = [' num2str(G(1)) ',' num2str(G(2)) ',' num2str(G(3)) '] and ' ...
            %    'tau_p = [' num2str(tau_p_net(1)) ',' num2str(tau_p_net(2)) ',' num2str(tau_p_net(3)) '].']);
            
            % Debug
            %printVector(G, 'G');
            %printVector(tau_p_net, 'tau_p_net');
            %printVector(e, 'e');
        end
        
        %__________________________________________________________________
        % Calculate the ESB torque for a given pose and psi
        function [ tau_p, tau_p_net ] = calc_tau_p(this, q, x)
            % Initialise ESB torque
            tau_p = zeros(this.model.leg.N/2);
            
            % Calculate the torque generated by each actuator
            it = 1;
            for i=1:size(this.params.T,1)
                % Find the type of actuator to calculate its torque
                if (strcmp(this.params.articulation{i}, 'noESB'))
                    % Do nothing, no ESB parameters to optimize for noESB
                    
                elseif (strcmp(this.params.articulation{i}, 'mono'))
                    psi_1 = x(it);
                    psi_2 = x(it+1);
                 	tau_p(i,:) = this.calc_tau_p_psi(q, i, psi_1, psi_2);
                    it = it+2;
                    
                elseif (strcmp(this.params.articulation{i}, 'bi'))
                    psi_1 = x(it);
                    psi_2 = x(it+1);
                    psi_3 = x(it+2);
                    tau_p(i,:) = this.calc_tau_p_biarticulated_psi(q, i, psi_1, psi_2, psi_3);
                    it = it+3;
                    
                elseif (strcmp(this.params.articulation{i}, 'tri'))
                    % TODO
                    
                else
                    error('articulation value error');
                end
            end

            % Calculate net ESB torque vector for this pose
            % (sum the columns)
            for i=1:size(tau_p,2)
                tau_p_net(i) = sum(tau_p(:,i));
            end
        end
        
        %__________________________________________________________________
        % Calculate the ESB torque from pretension and link positions (given a set
        % of parameters: psi_1, psi_2, psi_3),
        % for an alternative contept where the ESB spring runs over two pulleys,
        % where the first is a free pulley about joint 1, and joint 2 is actuated.
        function [ tau_p ] = calc_tau_p_biarticulated_psi(this, q, idx, psi_1, psi_2, psi_3)
            % Get this actuator's topology vector
            t = this.params.T(idx,:);
            
            % Calculate ESB torques
            % psi_1 = sqrt(k_p) * r_i
            % psi_2 = sqrt(k_p) * r_j
            % psi_3 = sqrt(k_p) * p
            % tau_p,i = psi_1 * psi_3 - psi_1^2 * q_i - psi_1 * psi_2 * q_j
            % tau_p,j = psi_2 * psi_3 - psi_1 * psi_2 * q_i - psi_2^2 * q_j
            % where the minus signs reduce the torque, following increases
            % in q_i, q_j, e.g. these joints reduce the elongation (e.g.
            % the biarticulated ankle+knee configuration from the RA-L
            % paper)
            
            % Here, we use the topology vector twice; first to calculate
            % the torque, then to map the torque back to the joint. This
            % works because t contains only -1, 0 or +1 (as the pulley
            % radii are contained in the psi's)
            
            % Create augmented topology vector from parameters
            j=0; % Counter to keep track of whether we are at psi_1 or psi_2
            % Turn something like [0 -1 1] into [0 -psi_1 psi_2]
            for i=1:length(t)
               if (t(i) ~= 0)
                   if (j == 0)
                       t(i) = t(i) * psi_1;
                       j = j+1;
                   else
                       t(i) = t(i) * psi_2;
                       j = j+1;
                   end
               end
            end
            
            tau_p = -t' * (psi_3 + t * q);

            % Make unidirectional
            tau_p(sign(tau_p) ~= -sign(t)') = 0;
        end
        
        %__________________________________________________________________
        % Calculate the ESB torque from pretension and link positions (given a set
        % of parameters psi_1 and psi_2)
        function [ tau_p ] = calc_tau_p_psi(this, q, idx, psi_1, psi_2)
            % Get this actuator's topology vector
            t = this.params.T(idx,:);
            
            % Calculate ESB torque
            % psi_1 = r * k_p * p
            % psi_2 = r^2 * k_p
            % tau_p = -psi_1 - psi_2 * q
            % where the minus signs on both terms shows that positive
            % elongation results in negative torque, and an increase in q
            % increases torque in that direction (e.g. monoarticulated knee
            % from RA-L paper)
            
            % Here, we use the topology vector twice; first to calculate
            % the torque, then to map the torque back to the joint. This
            % works because t contains only -1, 0 or +1 (as the pulley
            % radii are contained in the psi's)
            tau_p	= -t' * (psi_1 + psi_2 * t * q);
            
            % Make unidirectional
            tau_p(sign(tau_p) ~= -sign(t)') = 0;
        end
        
%         %__________________________________________________________________
%         % Calculate the ESB torque from pretension and link positions (given a set
%         % of parameters: pulley radius r_k [m] and stiffness k_p [N/m])
%         function [ tau_p ] = calc_tau_p(this, q, p, r, k_p)
%             % Calculate ESB torques
%             dL_p	= p + r * q;        % Calculate elongation
%             tau_p	= k_p * r * dL_p; 	% Calculate torque
% 
%             % TODO: Make unidirectional later!
%         end
%         
%         %__________________________________________________________________
%         % Calculate the ESB torque from pretension and link positions (given a set
%         % of parameters: pulley radius r_k [[m],[m]] and stiffness k_p [N/m]),
%         % for an alternative contept where the ESB spring runs over two pulleys,
%         % where the first is a free pulley about joint 1, and joint 2 is actuated.
%         function [ tau_p ] = calc_tau_p_biarticulated(this, q, p, params)
%             % Get parameters
%             k_p         = params.k_p_bi;    % ESB spring stiffness [N / m]
%             r_k_1       = params.r_k_bi(1);	% Actuated joint pulley radius [m]
%             r_k_2       = params.r_k_bi(2);	% Spanned joint free pulley radius [m]
% 
%             % Get variables
%             q_1 = q(1,:);
%             q_2 = q(2,:);
% 
%             % Calculate ESB torque
%             dL_p        = p - r_k_1 * q_1 - r_k_2 * q_2;    % Calculate elongation
%             tau_p_1     = k_p * r_k_1 * dL_p;               % Calculate torque on joint 1
%             tau_p_2     = k_p * r_k_2 * dL_p;               % Calculate torque on joint 2
% 
%             % Make unidirectional
%             % TODO FIX
%             %tau_p_1(tau_p_1 < 0) = 0;
%             %tau_p_2(tau_p_2 < 0) = 0;
% 
%             % Build result
%             tau_p = [tau_p_1; tau_p_2];
% 
%         end

        %__________________________________________________________________
    	% Calculate (augmented) topology matrix
        function [ T ] = topologyFromActuatorParams(this, actuatorParams)
            % Get non-augmented topology matrix
            T = this.params.T;
            
            % Cycle through all actuators
            for i=1:length(actuatorParams)
                t = T(i,:); % Get this actuator's topology vector
                
                % Get actuator type
                if (strcmp(actuatorParams{i}.type, 'noESB'))
                    % Do nothing, no ESB parameters for noESB

                elseif (strcmp(actuatorParams{i}.type, 'mono'))
                    % Multiply the nonzero element by r
                    t = t * actuatorParams{i}.r;

                elseif (strcmp(actuatorParams{i}.type, 'bi'))
                    k=0;
                    for j=1:length(t)
                       if (t(j) ~= 0)
                           if (k == 0)
                               t(j) = t(j) * actuatorParams{i}.r_bi_1;
                               k = k+1;
                           else
                               t(j) = t(j) * actuatorParams{i}.r_bi_2;
                               k = k+1;
                           end
                       end
                    end

                elseif (strcmp(actuatorParams{i}.type, 'tri'))
                    % TODO

                else
                    error('articulation value error');
                end
                
                % Substitute the now-augmented topology vector t back into
                % the topology matrix T
                T(i,:) = t;
            end
        end

    end
    
end

