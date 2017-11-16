classdef Leg_3DoF_ACA_jumpref_simulator < handle
    % Leg_3DoF_ACA_simulator - Simulator class for the 3-DoF leg driven by 3 ACAs
    
    %__________________________________________________________________
    % Properties
    properties
       model        % Leg_3DoF_ACA_jumpref model
       params       % Simulation parameters
       data         % Simulation data
       plots        % Plotting parameters & colours
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_ACA_jumpref_simulator(actParamsFileName, legParamsFileName)
            % Default parameters
            if (~exist('actParamsFileName', 'var'))
                actParamsFileName = '';
            end
            if (~exist('legParamsFileName', 'var'))
                legParamsFileName = '';
            end
            
            % Get object model
            this.model =  Leg_3DoF_ACA_jumpref(actParamsFileName, legParamsFileName);
    
            % Simulation parameters
            this.params.tspan       = [0 1];                   % Simulation time [s]
            this.params.Ts          = 1e-3;                     % Simulation timestep
            this.model.control.Ts	= 1e-3;                     % Controller timestep
            
            % Plotting parameters
            % Colours
            this.plots.C.R        	= [1.0 0.4 0.4];
            this.plots.C.G        	= [0.0 0.7 0.0]; % Old green: [0.3 0.8 0.3]
            this.plots.C.B       	= [0.3 0.3 1.0];
            this.plots.C.P       	= [0.8 0.0 1.0];
            this.plots.C.BLK        = [0.3 0.3 0.3];
            this.plots.C_light.R  	= [1.0 0.6 0.6];
            this.plots.C_light.G  	= [0.2 0.8 0.2];
            this.plots.C_light.B  	= [0.6 0.6 1.0];
            this.plots.C_light.P 	= [0.8 0.4 1.0];
            this.plots.C_light.GREY = [0.6 0.6 0.6];
            %0.5, 0.5, 0.0; % Additional colours
            %1.0, 0.7, 0.2;
            %0.0, 0.7, 0.7   ];
            % Other plotting parameters
            this.plots.filtIgnoreTime       = 1.0;
            this.plots.yAxisIgnoreTime      = 0.5;
            this.plots.sizeNormal           = [600 400];
            this.plots.sizePaperMode        = [500 190];%[600 230];
            this.plots.showTitleInPaperMode = 0;
            
            % Initialise empty simulation data
            this.data = struct;
            
        end
        
        %__________________________________________________________________
        % Run simulation
        function [] = run(this, interactive)
            % Default arguments
            if (~exist('interactive', 'var'))
                interactive = 1; % Interactive on by default
            end
            
            % Check timesteps
            if (    this.params.Ts > this.model.control.Ts || ...       % Simulation timestep smaller than control timestep
                    mod(this.model.control.Ts, this.params.Ts) ~= 0   )	% Controller timestep multiple of simulation timestep
                error('Invalid timesteps.');
            end
            
            % Build time array
            this.params.t =	this.params.tspan(1) : ...
                            this.params.Ts : ...
                            this.params.tspan(2);	% Timestep array (can be used instead of tspan for speed)
            
            % Initial conditions x0
            x0      = [ this.model.a1.x0; this.model.a2.x0; ...
                        this.model.a3.x0; this.model.leg.x0   ];
            
            % Simulate
            disp(['Simulating for ' num2str(this.params.tspan(2),'%3.1f') ' s...']);
            tic
            x = this.ode4int(@(t,x) this.model.dx_ode(t,x), this.params.t, x0, @(t,x) this.outFun(t, x));
            toc
            t = this.params.t;

            % Get state derivatives, outputs and inputs over state evolution
            % Get references as well
            disp('Reprocessing states to obtain outputs, inputs, and references..');
            tic
            
            % Preallocate variables
            y       = zeros(length(t), this.model.a1.K+this.model.a2.K+this.model.a3.K+this.model.leg.K);
            u       = zeros(length(t), this.model.a1.M+this.model.a2.M+this.model.a3.M+this.model.leg.M);
            tau_ref = zeros(length(t), this.model.leg.M);
            q_ref   = zeros(this.model.leg.N/2, length(t));
            
            % Go through all timesteps
            for i=1:length(t)
                [ ~, y(i,:), u(i,:), tau_ref(i,:) ]     = this.model.dx(t(i),x(i,:)');
                
                % Check whether we are using the random reference
                if (this.model.ref.use_random == 1)
                    q_ref(:,i)          = this.model.ref.random.q_ref(:,i);
                else
                    [q_ref(:,i), ~]     = this.model.q_ref(t(i));
                end
            end
            toc

            % Transpose because MATLAB has weird conventions
            t = t';

            % Get the actuator and leg states
            x_a1	= x(:,1:this.model.a1.N)';
            x_a2	= x(:,this.model.a1.N+1 : this.model.a1.N+this.model.a2.N)';
            x_a3	= x(:,this.model.a1.N+this.model.a2.N+1 : this.model.a1.N+this.model.a2.N+this.model.a3.N)';
            x_leg	= x(:,this.model.a1.N+this.model.a2.N+this.model.a3.N+1 : this.model.a1.N+this.model.a2.N+this.model.a3.N+this.model.leg.N)';

            % Get the actuator and leg outputs
            y_a1	= y(:,1:this.model.a1.K)';
            y_a2	= y(:,this.model.a1.K+1 : this.model.a1.K+this.model.a2.K)';
            y_a3	= y(:,this.model.a1.K+this.model.a2.K+1 : this.model.a1.K+this.model.a2.K+this.model.a3.K)';
            y_leg	= y(:,this.model.a1.K+this.model.a2.K+this.model.a3.K+1 : this.model.a1.K+this.model.a2.K+this.model.a3.K+this.model.leg.K)';

            % Get the actuator and leg inputs
            u_a1	= u(:,1:this.model.a1.M)';
            u_a2	= u(:,this.model.a1.M+1 : this.model.a1.M+this.model.a2.M)';
            u_a3	= u(:,this.model.a1.M+this.model.a2.M+1 : this.model.a1.M+this.model.a2.M+this.model.a3.M)';
            u_leg	= u(:,this.model.a1.M+this.model.a2.M+this.model.a3.M+1 : this.model.a1.M+this.model.a2.M+this.model.a3.M+this.model.leg.M)'; 

            % Get torque outputs
            tau_pb_1    = y_a1(this.model.a1.outputIdx_tau_pb(1),:);
            tau_pb_2    = y_a2(this.model.a2.outputIdx_tau_pb(2),:);
            tau_pb_3	= y_a3(this.model.a3.outputIdx_tau_pb(3),:);
            tau_p_a1   	= y_a1(this.model.a1.outputIdx_tau_p,:);
            tau_p_a2   	= y_a2(this.model.a2.outputIdx_tau_p,:);
            tau_p_a3   	= y_a3(this.model.a3.outputIdx_tau_p,:);
            tau_pb      = tau_pb_1 + tau_pb_2 + tau_pb_3;
            tau_p       = tau_p_a1 + tau_p_a2 + tau_p_a3;
            tau         = tau_p + [tau_pb_1; tau_pb_2; tau_pb_3];
            
            % Get motor powers
            P_A11       = y_a1(this.model.a1.outputIdx_P_1,:);
            P_A12       = y_a1(this.model.a1.outputIdx_P_2,:);
            P_A21       = y_a2(this.model.a2.outputIdx_P_1,:);
            P_A22       = y_a2(this.model.a2.outputIdx_P_2,:);
            P_A31       = y_a3(this.model.a3.outputIdx_P_1,:);
            P_A32       = y_a3(this.model.a3.outputIdx_P_2,:);
            
            % Restrict to positive (delivered) power in _pos vars
            P_A11_pos = P_A11;
            P_A12_pos = P_A12;
            P_A21_pos = P_A21;
            P_A22_pos = P_A22;
            P_A31_pos = P_A31;
            P_A32_pos = P_A32;
            P_A11_pos(P_A11_pos<0) = 0;
            P_A12_pos(P_A12_pos<0) = 0;
            P_A21_pos(P_A21_pos<0) = 0;
            P_A22_pos(P_A22_pos<0) = 0;
            P_A31_pos(P_A31_pos<0) = 0;
            P_A32_pos(P_A32_pos<0) = 0;
            
            % Sum motors to get actuator power and actuators to get system power
            P_A1        = P_A11_pos + P_A12_pos;
            P_A2        = P_A21_pos + P_A22_pos;
            P_A3        = P_A31_pos + P_A32_pos;
            P           = P_A1 + P_A2 + P_A3;
            
            % Calculate cumulative electrical energy consumption
            E_A1    = this.params.Ts * P_A1;
            E_A2    = this.params.Ts * P_A2;
            E_A3    = this.params.Ts * P_A3;
            E       = this.params.Ts * P;
            for i=2:length(P)
                E_A1(i)     = E_A1(i-1) + E_A1(i);
                E_A2(i)     = E_A2(i-1) + E_A2(i);
                E_A3(i)     = E_A3(i-1) + E_A3(i);
                E(i)        = E(i-1) + E(i);
            end
            
            % Get SS for state/output names
            %sys = this.model.a1.getStateSpace(); % unused

            % Set simulation output data
            this.data.t         = t;
            this.data.q_ref     = q_ref;
            this.data.x_a1      = x_a1;
            this.data.x_a2      = x_a2;
            this.data.x_a3      = x_a3;
            this.data.x_leg     = x_leg;
            this.data.y_a1      = y_a1;
            this.data.y_a2      = y_a2;
            this.data.y_a3      = y_a3;
            this.data.y_leg     = y_leg;
            this.data.u_a1      = u_a1;
            this.data.u_a2      = u_a2;
            this.data.u_a3      = u_a3;
            this.data.u_leg     = u_leg;
            this.data.tau_ref	= tau_ref'; % Transpose
            this.data.tau_pb    = tau_pb;
            this.data.tau_p     = tau_p;
            this.data.tau       = tau;
            this.data.E_A1      = E_A1;
            this.data.E_A2      = E_A2;
            this.data.E_A3      = E_A3;
            this.data.E         = E;
            
            % Ask for animation if interactive mode is on
            if (interactive)
                % Pause before animation
                disp('Press ENTER to animate or ctrl-C to quit immediately..');
                pause;

                % Quickly animate result
                this.animate();
            end
            
            % Finish
            disp('Finished.');
            
        end
        
        %__________________________________________________________________
        % Custom Runge-Kutta 4 based integrator (see ode4.m)
        % Compared to the original, we add an output function (similar to
        % ode45()'s outputFcn), and remove the varargin that gives
        % additional arguments to odefun (as this can be easily done using
        % anonymous functions).
        function Y = ode4int(this, odefun, tspan, y0, outputFcn)
            % Check tspan
            if ~isnumeric(tspan)
              error('TSPAN should be a vector of integration steps.');
            end

            % Check y0
            if ~isnumeric(y0)
              error('Y0 should be a vector of initial conditions.');
            end

            % Check tspan ordering
            h = diff(tspan);
            if any(sign(h(1))*h <= 0)
              error('Entries of TSPAN are not in order.') 
            end

            % Check odefun
            try
              f0 = feval(odefun,tspan(1),y0);
            catch
              msg = ['Unable to evaluate the ODEFUN at t0,y0. ',lasterr];
              error(msg);  
            end

            % Check y0, odefun consistency
            y0 = y0(:); % Make a column vector.
            if ~isequal(size(y0),size(f0))
              error('Inconsistent sizes of Y0 and f(t0,y0).');
            end  

            % Preallocation
            neq = length(y0);
            N = length(tspan);
            Y = zeros(neq,N);
            F = zeros(neq,4);

            % Set y(0) for t(0) calculated previously
            Y(:,1) = y0;
            
            % Integrate
            for i = 2:N
                ti = tspan(i-1);
                hi = h(i-1);
                yi = Y(:,i-1);
                F(:,1) = feval(odefun,ti,yi);
                F(:,2) = feval(odefun,ti+0.5*hi,yi+0.5*hi*F(:,1));
                F(:,3) = feval(odefun,ti+0.5*hi,yi+0.5*hi*F(:,2));  
                F(:,4) = feval(odefun,tspan(i),yi+hi*F(:,3));
                Y(:,i) = yi + (hi/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));
              
                % Run outputFcn
                try
                  stop = feval(outputFcn,ti,yi);
                catch
                  msg = ['Unable to evaluate the outputFcn at ti,yi. ',lasterr];
                  error(msg);  
                end
                if (stop)
                    break
                end
            end
            
            % Set result
            Y = Y.';
        end
        
        %__________________________________________________________________
        % Output function to abort if fallen over and plot leg state
        function stop = outFun(this, t, x) %#ok<*INUSD>
            % Don't stop by default
            stop = 0;
            
            % Show progress
            progress = 100 * (t / this.params.tspan(2));
            if (mod(progress, 5) == 0) % Show every 5%
                disp(['Progress: ' num2str(progress) '%']);
            end

            % Stop simulation if the leg has fallen over
            % x = [x_a1; x_a2; x_a3; x_leg]
            % x_leg = [q; q_d]
            % q = [x1, y1, theta1, q1, q2, q3]
            theta1 = x(end-this.model.leg.N+3);	% Get foot angle q(3)=theta1
            if (theta1 > pi/2 || theta1 < -pi/2)
                disp('Leg has fallen over. Stopping simulation.');
                stop = 1;
            end

            % Draw every n seconds
            n = inf;%0.1; % inf for no plots
            n = n - mod(n, this.params.Ts); % Round to nearest multiple of Ts
            doPlot = 0;
            if (mod(t, n) == 0)
                doPlot = 1;
            end

            % Check if we should plot
            if (doPlot)        
                % Draw leg in its current configuration
                figure(1337); clf; hold on; grid on;
                axis equal;
                draw_configuration(this.model.leg, q, t(end)); % Draw configuration
            end
        end
        
        %__________________________________________________________________
        % Plot simulation results
        function [] = plot(this)
            % First ask whether to save plots
            paperMode = confirm('Generate plots in paper mode? [y/N]', 0);
            savePlots = confirm('Save plots as PDF? [y/N]', 0);
            
            % Get plot size depending on paperMode
            if (paperMode)
                figSize = this.plots.sizePaperMode;
            else
                figSize = this.plots.sizeNormal;
            end
            
            % Get some plotting parameters
            filtIgnoreTime          = this.plots.filtIgnoreTime;
            yAxisIgnoreTime         = this.plots.yAxisIgnoreTime;
            showTitleInPaperMode    = this.plots.showTitleInPaperMode;
            C                       = this.plots.C;         % Plot colours
            C_light                 = this.plots.C_light;   % Light plot colours
            
            % Get variables from data
            % leg: q = [x1, y1, theta1, q1, q2, q3]
            % leg: x = [q; q_d]
            t               = this.data.t;
            q_ref           = this.data.q_ref;
            x_a1            = this.data.x_a1;
            x_a2            = this.data.x_a2;
            x_a3            = this.data.x_a3;
            x_leg           = this.data.x_leg;
            q_leg           = x_leg(1:this.model.leg.N/2,:);
            q_leg_d         = x_leg(this.model.leg.N/2 + 1:end,:);
            q_joints        = q_leg(4:6,:);
            q_joints_d      = q_leg_d(4:6,:);
            x_floatingBase  = [q_leg(1:3,:); q_leg_d(1:3,:)];
            y_a1            = this.data.y_a1;
            y_a2            = this.data.y_a2;
            y_a3            = this.data.y_a3;
            y_leg           = this.data.y_leg;
            u_a1            = this.data.u_a1;
            u_a2            = this.data.u_a2;
            u_a3            = this.data.u_a3;
            u_leg           = this.data.u_leg;
            tau_ref         = this.data.tau_ref;
            E_A1            = this.data.E_A1;
            E_A2            = this.data.E_A2;
            E_A3            = this.data.E_A3;
            E               = this.data.E;
            
            % If saving plots, resample the data to a lower resolution
            sampleTime  = this.params.Ts;
            plotPath    = 'plots/';
            if (savePlots)
                % Create new time vector
                sampleTime  = 0.01;
                t_RS        = [0:sampleTime:max(t)]'; %#ok<NBRAK>
                
                % Resample data
                q_ref           = interp1(t, q_ref',            t_RS)';
                x_a1            = interp1(t, x_a1',             t_RS)';
                x_a2            = interp1(t, x_a2',             t_RS)';
                x_a3            = interp1(t, x_a3',             t_RS)';
                x_leg           = interp1(t, x_leg',            t_RS)';
                q_leg           = interp1(t, q_leg',            t_RS)';
                q_leg_d         = interp1(t, q_leg_d',          t_RS)';
                q_joints        = interp1(t, q_joints',         t_RS)';
                q_joints_d      = interp1(t, q_joints_d',       t_RS)';
                x_floatingBase  = interp1(t, x_floatingBase',   t_RS)';
                y_a1            = interp1(t, y_a1',             t_RS)';
                y_a2            = interp1(t, y_a2',             t_RS)';
                y_a3            = interp1(t, y_a3',             t_RS)';
                y_leg           = interp1(t, y_leg',            t_RS)'; %#ok<*NASGU>
                u_a1            = interp1(t, u_a1',             t_RS)';
                u_a2            = interp1(t, u_a2',             t_RS)';
                u_a3            = interp1(t, u_a3',             t_RS)';
                u_leg           = interp1(t, u_leg',            t_RS)';
                tau_ref         = interp1(t, tau_ref',          t_RS)';
                E_A1            = interp1(t, E_A1',             t_RS)';
                E_A2            = interp1(t, E_A2',             t_RS)';
                E_A3            = interp1(t, E_A3',             t_RS)';
                E               = interp1(t, E',                t_RS)';
                
                % Overwrite old time vector so that everything below works
                t           = t_RS;
                
                % Make sure plotting path exists
                if (~exist(plotPath, 'dir'))
                   mkdir(plotPath); 
                end
            end
            
            % Get some outputs
            tau_pb_1    = y_a1(this.model.a1.outputIdx_tau_pb(1),:);
            tau_pb_2    = y_a2(this.model.a2.outputIdx_tau_pb(2),:);
            tau_pb_3	= y_a3(this.model.a3.outputIdx_tau_pb(3),:);
            tau_p_a1   	= y_a1(this.model.a1.outputIdx_tau_p,:);
            tau_p_a2   	= y_a2(this.model.a2.outputIdx_tau_p,:);
            tau_p_a3   	= y_a3(this.model.a3.outputIdx_tau_p,:);
            tau_p       = tau_p_a1 + tau_p_a2 + tau_p_a3;
            tau         = tau_p + [tau_pb_1; tau_pb_2; tau_pb_3]; % = this.data.tau, see run()
            
            % Get/calculate some other quantities
            
            % Get motor powers
            P_A11       = y_a1(this.model.a1.outputIdx_P_1,:);
            P_A12       = y_a1(this.model.a1.outputIdx_P_2,:);
            P_A21       = y_a2(this.model.a2.outputIdx_P_1,:);
            P_A22       = y_a2(this.model.a2.outputIdx_P_2,:);
            P_A31       = y_a3(this.model.a3.outputIdx_P_1,:);
            P_A32       = y_a3(this.model.a3.outputIdx_P_2,:);
            
            % Restrict to positive (delivered) power in _pos vars
            P_A11_pos = P_A11;
            P_A12_pos = P_A12;
            P_A21_pos = P_A21;
            P_A22_pos = P_A22;
            P_A31_pos = P_A31;
            P_A32_pos = P_A32;
            P_A11_pos(P_A11_pos<0) = 0;
            P_A12_pos(P_A12_pos<0) = 0;
            P_A21_pos(P_A21_pos<0) = 0;
            P_A22_pos(P_A22_pos<0) = 0;
            P_A31_pos(P_A31_pos<0) = 0;
            P_A32_pos(P_A32_pos<0) = 0;
            
            % Sum motors to get actuator power and actuators to get system power
            P_A1        = P_A11_pos + P_A12_pos;
            P_A2        = P_A21_pos + P_A22_pos;
            P_A3        = P_A31_pos + P_A32_pos;
            P           = P_A1 + P_A2 + P_A3;
            
            % Filter powers
            if (~this.model.ref.use_random)
                % For a non-random (cyclic) reference, use the period
                windowSize	= this.model.ref.T / sampleTime;
                b           = ones(1,windowSize) / windowSize;
                a           = 1;
            else
                % Otherwise, use a fixed number of seconds
                windowSize	= 8 / sampleTime;
                b           = ones(1,windowSize) / windowSize;%linspace(2/windowSize,0,windowSize);
                a           = 1;
            end
            P_A1_filt   = P_A1;
            P_A2_filt   = P_A2;
            P_A3_filt   = P_A3;
            P_filt      = P;
            N           = 1;
            % Ignoring the first filtIgnoreTime seconds due to startup behaviour
            if (filtIgnoreTime > 0)
               N                    = round(filtIgnoreTime / sampleTime);
               P_A1_filt(1:N-1)     = P_A1_filt(N);
               P_A2_filt(1:N-1)     = P_A2_filt(N);
               P_A3_filt(1:N-1)     = P_A3_filt(N);
               P_filt(1:N-1)        = P_filt(N);
            end
            % Do actual filtering
            P_A1_filt   = filter(b, a, P_A1_filt, linspace(1,0,windowSize-1) * P_A1_filt(N));
            P_A2_filt   = filter(b, a, P_A2_filt, linspace(1,0,windowSize-1) * P_A2_filt(N));
            P_A3_filt   = filter(b, a, P_A3_filt, linspace(1,0,windowSize-1) * P_A3_filt(N));
            P_filt      = filter(b, a, P_filt, linspace(1,0,windowSize-1) * P_filt(N));
            
            % Energy stored inside ESBs
            dL_p_1      = x_a1(this.model.a1.stateIdx_dL_p,:);
            dL_p_2      = x_a2(this.model.a2.stateIdx_dL_p,:);
            dL_p_3      = x_a3(this.model.a3.stateIdx_dL_p,:);
            dL_p_1(dL_p_1 < 0) = 0;
            dL_p_2(dL_p_2 < 0) = 0;
            dL_p_3(dL_p_3 < 0) = 0;
            E_p_1       = 0.5 * this.model.a1.params.k_p * dL_p_1.^2;
            E_p_2       = 0.5 * this.model.a2.params.k_p * dL_p_2.^2;
            E_p_3       = 0.5 * this.model.a3.params.k_p * dL_p_3.^2;
            
            % Motor voltages
            v11         = y_a1(this.model.a1.outputIdx_v_1,:);
            v12         = y_a1(this.model.a1.outputIdx_v_2,:);
            v21         = y_a2(this.model.a2.outputIdx_v_1,:);
            v22         = y_a2(this.model.a2.outputIdx_v_2,:);
            v31         = y_a3(this.model.a3.outputIdx_v_1,:);
            v32         = y_a3(this.model.a3.outputIdx_v_2,:);
            
            % Do plots
            
            % Leg joints/tracking
            figure(1); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, q_joints(1,:), 'Color', C.B);
            plot(t, q_joints(2,:), 'Color', C.R);
            plot(t, q_joints(3,:), 'Color', C.G);
            plot(t, q_ref(4,:), '--', 'Color', C.B);
            plot(t, q_ref(5,:), '--', 'Color', C.R);
            plot(t, q_ref(6,:), '--', 'Color', C.G);
            setYAxis;
            paperModeLegend(    paperMode, ...
                                {'$q_1$', '$q_2$', '$q_3$', '$q_1^*$', '$q_2^*$', '$q_3^*$'},   ...
                                {'q_1', 'q_2', 'q_3', 'q_{1,ref}', 'q_{2,ref}', 'q_{3,ref}'}    );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[rad]'}, {'t [s]', '[rad]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Leg joints/tracking',  ...
                                'Leg joints/tracking'   );
            paperSave(savePlots, [plotPath 'leg_joints.pdf']);
            
            % Joint velocities
            figure(2); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, q_joints_d(1,:), 'Color', C.B);
            plot(t, q_joints_d(2,:), 'Color', C.R);
            plot(t, q_joints_d(3,:), 'Color', C.G);
            setYAxis(-1, 2*yAxisIgnoreTime);
            paperModeLegend(    paperMode, ...
                                {'$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$'},    ...
                                {'q_{1,d}', 'q_{2,d}', 'q_{3,d}'}           );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[rad/s]'}, {'t [s]', '[rad/s]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Joint velocities',  ...
                                'Joint velocities'   );
            paperSave(savePlots, [plotPath 'leg_joint_velocities.pdf']);
            
            % Floating base velocities
            figure(3); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, x_floatingBase);
            setYAxis(-1, 2*yAxisIgnoreTime);
            paperModeLegend(    paperMode, ...
                                {'$x_1$ [m]', '$y_1$ [m]', '$\theta_1$ [rad]', '$\dot{x}_1$ [m/s]', '$\dot{y}_1$ [m/s]', '$\dot{\theta}_1$ [rad/s]'},   ...
                                {'x_1 [m]', 'y_1 [m]', 'theta_1 [rad]', 'x_{1,d} [m/s]', 'y_{1,d} [m/s]', 'theta_{1,d} [rad/s]'}    );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[]'}, {'t [s]', '[]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Floating base configuration and velocity',  ...
                                'Floating base configuration and velocity'   );
            paperSave(savePlots, [plotPath 'floating_base.pdf']);

            % Joint 1 torques
            figure(4); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, tau_ref(1,:), '--', 'Color', C.BLK);
            plot(t, tau_pb_1, 'Color', C.B);
            if (    strcmpi(this.model.a1.type, 'bi') || ...
                    strcmpi(this.model.a1.type, 'mono')         )
                plot(t, tau_p_a1(1,:), 'Color', C.R);
                plot(t, tau(1,:), 'Color', C.BLK);
                paperModeLegend(    paperMode, ...
                                    {'$\tau_1^*$', '$\tau_{PB,A1}$', '$\tau_{p,A1}$', '$\tau_1$'},   ...
                                    {'tau_{1,ref}', 'tau_{PB,A1}', 'tau_{p,A1}', 'tau_1'}                );
              	paperModeTitle(     showTitleInPaperMode, paperMode, ...
              	                    'Joint 1 torques',  ...
              	                    'Joint 1 torques'   );
            else
                paperModeLegend(    paperMode, ...
                                    {'$\tau_1^*$', '$\tau_{PB,1}$'},    ...
                                    {'tau_{1,ref}', 'tau_{PB,1}'}           );
                paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                    'Joint 1 torques (noESB)',  ...
                                    'Joint 1 torques (noESB)'   );
            end
            setYAxis(-1, yAxisIgnoreTime);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[N m]'}, {'t [s]', '[N m]'});
            paperSave(savePlots, [plotPath 'q_1_torques.pdf']);
            
            % Joint 2 torques
            figure(5); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, tau_ref(2,:), '--', 'Color', C.BLK);
            plot(t, tau_pb_2, 'Color', C.B);
            if (strcmpi(this.model.a1.type, 'bi'))
                % Biarticulated
                plot(t, tau_p_a1(2,:), 'Color', C.P);
                plot(t, tau_p_a2(2,:), 'Color', C.G);
                plot(t, tau_p_a1(2,:)+tau_p_a2(2,:), 'Color', C.R);
                plot(t, tau(2,:), 'Color', C.BLK);
                paperModeLegend(    paperMode, ...
                                    {'$\tau_2^*$', '$\tau_{PB,A2}$', '$\tau_{p,A1}$', '$\tau_{p,A2}$', '$\tau_{p,A1}+\tau_{p,A2}$', '$\tau_2$'},     ...
                                    {'tau_{2,ref}', 'tau_{PB,A2}', 'tau_{p,A1}', 'tau_{p,A2}', 'tau_{p,a1}+tau_{p,A2}', 'tau_2'}                         );
                paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                    'Joint 2 torques (biarticulated)',  ...
                                    'Joint 2 torques (biarticulated)'   );
            elseif (strcmpi(this.model.a1.type, 'mono'))
                % monoarticulated
                plot(t, tau_p_a2(2,:), 'Color', C.R);
                plot(t, tau(2,:), 'Color', C.BLK);
                paperModeLegend(    paperMode, ...
                                    {'$\tau_2^*$', '$\tau_{PB,A2}$', '$\tau_{p,A2}$', '$\tau_2$'},   ...
                                    {'tau_{2,ref}', 'tau_{PB,A2}', 'tau_{p,A2}', 'tau_2'}                );
                paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                    'Joint 2 torques (monoarticulated)',  ...
                                    'Joint 2 torques (monoarticulated)'   );
            else
                % noESB
                paperModeLegend(    paperMode, ...
                                    {'$\tau_2^*$', '$\tau_{PB,A2}$'},    ...
                                    {'tau_{2,ref}', 'tau_{PB,A2}'}       	);
                paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                    'Joint 2 torques (noESB)',  ...
                                    'Joint 2 torques (noESB)'   );
            end
            setYAxis(-1, yAxisIgnoreTime);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[N m]'}, {'t [s]', '[N m]'});
            paperSave(savePlots, [plotPath 'q_2_torques.pdf']);
            
            % Joint 3 torques
            figure(6); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, tau_ref(3,:), '--', 'Color', C.BLK);
            plot(t, tau_pb_3, 'Color', C.B);
            %plot(t, tau(3,:), 'Color', C.BLK);
            setYAxis(-1, 2*yAxisIgnoreTime);
            paperModeLegend(    paperMode, ...
                            	{'$\tau_3^*$', '$\tau_{PB,A3}$'},    ...
                             	{'tau_{3,ref}', 'tau_{PB,A3}'}       	);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[N m]'}, {'t [s]', '[N m]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Joint 3 torques',  ...
                                'Joint 3 torques'   );
            paperSave(savePlots, [plotPath 'q_3_torques.pdf']);
            
            % ESB extension
            figure(7); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, x_a1(this.model.a1.stateIdx_dL_p,:), 'Color', C.B);
            plot(t, x_a2(this.model.a2.stateIdx_dL_p,:), 'Color', C.R);
            setYAxis;
            paperModeLegend(	paperMode, ...
                                {'$\Delta_{p,1}$', '$\Delta_{p,2}$'},   ...
                                {'dL_{p,1}', 'dL_{p,2}'}                );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[m]'}, {'t [s]', '[m]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'ESB extension',  ...
                                'ESB extension'   );
            paperSave(savePlots, [plotPath 'dL_p.pdf']);
            
            % Pretension positions
            figure(8); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, x_a1(this.model.a1.stateIdx_p,:), 'Color', C.B);
            plot(t, x_a2(this.model.a2.stateIdx_p,:), 'Color', C.R);
            plot([0 max(t)], [this.model.a1.params.p_range(1) this.model.a1.params.p_range(1)], '--', 'Color', C.B);
            plot([0 max(t)], [this.model.a1.params.p_range(2) this.model.a1.params.p_range(2)], '--', 'Color', C.B);
            plot([0 max(t)], [this.model.a2.params.p_range(1) this.model.a2.params.p_range(1)], '--', 'Color', C.R);
            plot([0 max(t)], [this.model.a2.params.p_range(2) this.model.a2.params.p_range(2)], '--', 'Color', C.R);
            setYAxis;
            paperModeLegend(	paperMode, ...
                                {'$p_1$', '$p_2$'},     ...
                                {'p_1', 'p_2'}      	);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[m]'}, {'t [s]', '[m]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Pretension positions',  ...
                                'Pretension positions'   );
            paperSave(savePlots, [plotPath 'p.pdf']);
            
            % Electrical power
            figure(9); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, P_A1, 'Color', C_light.B);
            plot(t, P_A2, 'Color', C_light.R);
            plot(t, P_A3, 'Color', C_light.G);
            plot(t, P, 'Color', C_light.GREY);
            h1 = plot(t, P_A1_filt, 'Color', C.B, 'LineWidth', 2);
            h2 = plot(t, P_A2_filt, 'Color', C.R, 'LineWidth', 2);
            h3 = plot(t, P_A3_filt, 'Color', C.G, 'LineWidth', 2);
            h4 = plot(t, P_filt, 'Color', C.BLK, 'LineWidth', 2);
            setYAxis(-1, yAxisIgnoreTime);
            paperModeLegend(	paperMode, ...
                                {   ['$P_{A1}$ filt. (' num2str(P_A1_filt(end), '%3.1f') ' W end)'],	...
                                    ['$P_{A2}$ filt. (' num2str(P_A2_filt(end), '%3.1f') ' W end)'],    ...
                                    ['$P_{A3}$ filt. (' num2str(P_A3_filt(end), '%3.1f') ' W end)'],    ...
                                    ['$P$ filt. (' num2str(P_filt(end), '%3.1f') ' W end)']             }, ...
                                {   'P_{A1}', 'P_{A2}', 'P_{A3}', 'P', ...
                                    ['P_{A1} filt. (' num2str(P_A1_filt(end), '%3.1f') ' W end)'],  ...
                                    ['P_{A2} filt. (' num2str(P_A2_filt(end), '%3.1f') ' W end)'],  ...
                                    ['P_{A3} filt. (' num2str(P_A3_filt(end), '%3.1f') ' W end)'],  ...
                                    ['P filt. (' num2str(P_filt(end), '%3.1f') ' W end)']           }, ...
                                    [h1,h2,h3,h4] );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[W]'}, {'t [s]', '[W]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Electrical power (positive only)',  ...
                                'Electrical power (positive only)'   );
            paperSave(savePlots, [plotPath 'P.pdf']);
            
            % Electrical power M1 vs M2 per actuator unit
            figure(10); clf;
            resizeFig(gcf, figSize(1), figSize(2));
            subplot(2,2,1); hold on; grid on;
            plot(t, P_A11, 'Color', C.B);
            plot(t, P_A12, 'Color', C.R);
            paperModeLegend(	paperMode, ...
                                {'$P_{m1,A1}$', '$P_{m2,A1}$'},   ...
                                {'P_{m1,A1}', 'P_{m2,A1}'}      	);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[W]'}, {'t [s]', '[W]'});
            setYAxis(-1, yAxisIgnoreTime);
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'PB vs ESB power: ACA1',  ...
                                'PB vs ESB power: ACA1'   );
            subplot(2,2,2); hold on; grid on;
            plot(t, P_A21, 'Color', C.B);
            plot(t, P_A22, 'Color', C.R);
            paperModeLegend(	paperMode, ...
                                {'$P_{m1,A2}$', '$P_{m2,A2}$'},   ...
                                {'P_{m1,A2}', 'P_{m2,A2}'}      	);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[W]'}, {'t [s]', '[W]'});
            setYAxis(-1, yAxisIgnoreTime);
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'PB vs ESB power: ACA2',  ...
                                'PB vs ESB power: ACA2'   );
            subplot(2,2,3); hold on; grid on;
            plot(t, P_A31, 'Color', C.B);
            plot(t, P_A32, 'Color', C.R);
            paperModeLegend(	paperMode, ...
                                {'$P_{m1,A3}$', '$P_{m2,A3}$'},   ...
                                {'P_{m1,A3}', 'P_{m2,A3}'}      	);
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[W]'}, {'t [s]', '[W]'});
            setYAxis(-1, yAxisIgnoreTime);
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'PB vs ESB power: ACA3',  ...
                                'PB vs ESB power: ACA3'   );
            paperSave(savePlots, [plotPath 'P_M1_vs_M2.pdf']);
            
            % Motor currents
            figure(11); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, u_a1(1,:), 'Color', C.B);
            plot(t, u_a1(2,:), '--', 'Color', C.B);
            plot(t, u_a2(1,:), 'Color', C.R);
            plot(t, u_a2(2,:), '--', 'Color', C.R);
            plot(t, u_a3(1,:), 'Color', C.G);
            plot(t, u_a3(2,:), '--', 'Color', C.G);
            paperModeLegend(	paperMode, ...
                                {'$i_{m1,A1}$', '$i_{m2,A1}$', '$i_{m1,A2}$', '$i_{m2,A2}$', '$i_{m1,A3}$', '$i_{m2,A3}$'},     ...
                                {'i_{m1,A1}', 'i_{m2,A1}', 'i_{m1,A2}', 'i_{m2,A2}', 'i_{m1,A3}', 'i_{m2,A3}'}                  );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[A]'}, {'t [s]', '[A]'});
            setYAxis(-1, yAxisIgnoreTime);
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Motor currents',  ...
                                'Motor currents'   );
            paperSave(savePlots, [plotPath 'currents.pdf']);
            
            % M2 rotor velocities
            rs2rpm = 60/(2*pi);
            figure(12); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, rs2rpm * x_a1(this.model.a1.stateIdx_p_d,:) * this.model.a1.params.r_m2, 'Color', C.B);
            plot(t, rs2rpm * x_a2(this.model.a2.stateIdx_p_d,:) * this.model.a2.params.r_m2, 'Color', C.R);
            plot(t, rs2rpm * x_a3(this.model.a3.stateIdx_p_d,:) * this.model.a3.params.r_m2, 'Color', C.G);
            paperModeLegend(	paperMode, ...
                                {'$\dot{p}_1$', '$\dot{p}_2$', '$\dot{p}_3$'},    ...
                                {'p_{d,1}', 'p_{d,2}', 'p_{d,3}'}           );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[rpm]'}, {'t [s]', '[rpm]'});
            setYAxis;
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'M2 rotor velocities',  ...
                                'M2 rotor velocities'   );
            paperSave(savePlots, [plotPath 'M2_speed.pdf']);
            
            % Energy stored inside ESBs
            figure(13); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, E_p_1, 'Color', C.B);
            plot(t, E_p_2, 'Color', C.R);
            plot(t, E_p_3, 'Color', C.G);
            paperModeLegend(	paperMode, ...
                                {'$E_{p,1}$', '$E_{p,2}$', '$E_{p,3}$'},    ...
                                {'E_{p,1}', 'E_{p,2}', 'E_{p,3}'}           );
            paperModeAxisLabels(paperMode, {'$t$ [s]', 'Energy [J]'}, {'t [s]', 'Energy [J]'});
            setYAxis;
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'ESB stored energy',  ...
                                'ESB stored energy'   );
            paperSave(savePlots, [plotPath 'ESB_storage.pdf']);
            
            % Motor voltages
            figure(14); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, v11, 'Color', C.B);
            plot(t, v12, '--', 'Color', C.B);
            plot(t, v21, 'Color', C.R);
            plot(t, v22, '--', 'Color', C.R);
            plot(t, v31, 'Color', C.G);
            plot(t, v32, '--', 'Color', C.G);
            paperModeLegend(	paperMode, ...
                                {'$v_{m1,A1}$', '$v_{m2,A1}$', '$v_{m1,A2}$', '$v_{m2,A2}$', '$v_{m1,A3}$', '$v_{m2,A3}$'},     ...
                                {'v_{m1,A1}', 'v_{m2,A1}', 'v_{m1,A2}', 'v_{m2,A2}', 'v_{m1,A3}', 'v_{m2,A3}'}                  );
            paperModeAxisLabels(paperMode, {'$t$ [s]', 'Voltage [V]'}, {'t [s]', 'Voltage [V]'});
            setYAxis(-1, yAxisIgnoreTime);
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Motor voltages',  ...
                                'Motor voltages'   );
            paperSave(savePlots, [plotPath 'voltages.pdf']);
            
            % Cumulative electrical energy consumption
            figure(15); clf; hold on; grid on;
            resizeFig(gcf, figSize(1), figSize(2));
            plot(t, E_A1, 'Color', C.B);
            plot(t, E_A2, 'Color', C.R);
            plot(t, E_A3, 'Color', C.G);
            plot(t, E, 'Color', C.BLK);
            setYAxis;
            paperModeLegend(	paperMode, ...
                                {'$E_{A1}$', '$E_{A2}$', '$E_{A3}$', '$E$'},    ...
                                {'E_{A1}', 'E_{A2}', 'E_{A3}', 'E'}             );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '[J]'}, {'t [s]', '[J]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Cumulative electrical energy consumption',  ...
                                'Cumulative electrical energy consumption'   );
            paperSave(savePlots, [plotPath 'E.pdf']);
            
            disp('Finished.');
        end
        
        
        %__________________________________________________________________
        % Animate the leg
        function [] = animate(this, t, x_leg, tau, infoText)
            % Default parameters
            if (~exist('t', 'var') || ~exist('x_leg', 'var') || ~exist('tau', 'var'))
                if (~isempty(this.data.t) && ~isempty(this.data.x_leg)  && ~isempty(this.data.tau))
                    % Get variables from simulation results
                    t       = this.data.t;
                    x_leg	= this.data.x_leg;
                    tau     = this.data.tau'; % Transpose
                else
                    error('Cannot animate, no simulation results found!');
                end
            end
            if (~exist('infoText', 'var'))
                infoText = '';
            end
            
            % Get leg configurations
            % x_leg = [q_leg; q_leg_d]
            q_leg = x_leg(1:this.model.leg.N/2,:);
         
            % Get hip x-y position from forward kinematics
            [x_hip, y_hip, ~] = this.model.leg.calc_fwdKin_named(q_leg, 'hip');
            
            % Animate
            figure; clf; hold on; axis square;
            for i=1:length(t)
                % Only plot every some steps
                if (mod(t(i), 50*this.params.Ts) ~= 0)
                    continue;
                end
                
                % Check joint limits
                if (	q_leg(4,i) < this.model.leg.params.q_limits(1,1)    || ...
                        q_leg(4,i) > this.model.leg.params.q_limits(1,2)    )
                    error(['Leg has hit joint limit on joint 1: q(4) = ' num2str(q_leg(1,i)) '!']);
                end
                if (	q_leg(5,i) < this.model.leg.params.q_limits(2,1)    || ...
                        q_leg(5,i) > this.model.leg.params.q_limits(2,2)    )
                    error(['Leg has hit joint limit on joint 2: q(5) = ' num2str(q_leg(2,i)) '!']);
                end
                if (	q_leg(6,i) < this.model.leg.params.q_limits(3,1)    || ...
                        q_leg(6,i) > this.model.leg.params.q_limits(3,2)    )
                    error(['Leg has hit joint limit on joint 3: q(6) = ' num2str(q_leg(3,i)) '!']);
                end

                % Draw configuration
                draw_configuration(     this.model.leg, ...
                                        q_leg(:,1:i), ...
                                        tau, ...
                                        t(i), ...
                                        infoText            );
                
                % Plot hip path up to this time
                plot(x_hip(1:i), y_hip(1:i));
                drawnow;
            end
            close 1;
        end
        
        %__________________________________________________________________
        % Animate the reference
        function [] = animateReference(this)
            if (this.model.ref.use_random == 1)
                % Animate random reference
                this.animate(   this.model.ref.random.t, ...
                                this.model.ref.random.q_ref, ...
                                'REFERENCE'                         );
                t       = this.model.ref.random.t;
                q_ref	= this.model.ref.random.q_ref;
            else
                % Create time vector, get and animate reference
                t = this.params.tspan(1) : this.params.Ts : this.params.tspan(2);
                [ q_ref, ~ ] = this.model.q_ref(t);
                this.animate(	t, ...
                                q_ref, ...
                                -1, ...
                                'REFERENCE'     );
            end
            
            % Get joint limits
            q_limits = this.model.leg.params.q_limits;
            
            % Show joint angles over time
            figure; grid on; hold on;
            h1 = plot(t, q_ref(4,:), 'Color', [0 0 1]);
            h2 = plot(t, q_ref(5,:), 'Color', [1 0 0]);
            h3 = plot(t, q_ref(6,:), 'Color', [0 1 0]);
            plot([0 max(t)], [q_limits(1,1) q_limits(1,1)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(1,2) q_limits(1,2)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(2,1) q_limits(2,1)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(2,2) q_limits(2,2)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(3,1) q_limits(3,1)], '--', 'Color', [0 0.8 0]);
            plot([0 max(t)], [q_limits(3,2) q_limits(3,2)], '--', 'Color', [0 0.8 0]);
            legend([h1 h2 h3], 'q_1', 'q_2', 'q_3');
            paperModeAxisLabels(0, {'t [s]', 'Joint angle reference [rad]'}, {'t [s]', 'Joint angle reference [rad]'});
            paperModeTitle(     1, 0, ...
                                'Joint angle references over time',  ...
                                'Joint angle references over time'   );
            drawnow;
        end
        
        %__________________________________________________________________
        % Animate the leg and export as a movie
        function [] = animateExport(this)
            % Check if simulation results exist
            if (isempty(this.data.t) || isempty(this.data.x_leg)  || isempty(this.data.tau))
                error('Cannot animate, no simulation results found!');
            end
            
            % Status
            disp('Writing leg animation to ''legAnimation.avi''..');
            
            % Get variables from simulation results
            t           = this.data.t;
            x_leg       = this.data.x_leg;
            tau         = this.data.tau'; % Transpose
            
            % Frame rate, time step
            fps         = this.model.leg.params.fps;
            Ts_RS       = this.model.leg.params.Ts_RS;
            
            % Resample
            t_RS        = 0:Ts_RS:max(t);
            x_leg_RS  	= interp1(t, x_leg', t_RS)';
            tau_RS    	= interp1(t, tau, t_RS);
            
            % Get leg configurations
            q_leg_RS	= x_leg_RS(1:this.model.leg.N/2,:);
            
            % Get hip x-y position from forward kinematics
            [x_hip, y_hip, ~] = this.model.leg.calc_fwdKin_named(q_leg_RS, 'hip');
            
            % Prepare VideoWriter
            v           = VideoWriter('legAnimation.avi');
            v.Quality	= 95;
            v.FrameRate = fps;
            open(v);
            
            % Animate
            h = figure;
            pos = get(gcf, 'Position');
            xPos = pos(1); yPos = pos(2);
            set(gcf, 'Position', [xPos yPos 400 400]);
            hold on; axis square;
            for i=1:length(t_RS)
                % Draw configuration
                draw_configuration(     this.model.leg, ...
                                        q_leg_RS(:,1:i), ...
                                        tau_RS, ...
                                        t_RS(i), ...
                                        ''              );
                
                % Plot hip path up to this time
                plot(x_hip(1:i), y_hip(1:i));
                
                % Draw now and store frame
                drawnow;
                M = getframe;
                writeVideo(v,M);
            end
            
            % Close figure and VideoWriter
            close(h);
            close(v);

            % Execute handbrake
            disp('Converting to MKV using HandBrakeCLI..');
            system(['HandBrakeCLI --input "' pwd '/legAnimation.avi" ' ...
                '--output "' pwd '/legAnimation.mkv" --preset "High Profile"']);

            % Delete AVI
            disp('Deleting AVI..');
            system(['rm "' pwd '/legAnimation.avi"']);
            
            disp('Finished.');
        end
        
    end
    
end

