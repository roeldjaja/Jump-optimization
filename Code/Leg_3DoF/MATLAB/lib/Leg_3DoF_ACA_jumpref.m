classdef Leg_3DoF_ACA_jumpref < handle
    
    %__________________________________________________________________
    % Properties
    properties
        leg     % Leg_3DoF object
        a1      % ACA object 1
        a2      % ACA object 2
        a3      % ACA object 3
        ref     % Reference parameters
        control % Control parameters/variables
        actParamsFileName % Actuation parameters filename
        p       % Pretension parameter
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_ACA_jumpref(actParamsFileName, legParamsFileName, quiet)
            % Set default arguments
            
            % actParamsFileName
            if (~exist('actParamsFileName', 'var'))
                actParamsFileName = '';
            elseif (~strcmp(actParamsFileName, '') && ~exist(actParamsFileName, 'file'))
                error(['ERROR: Actuation parameters file ''' actParamsFileName ''' not found!']);
            end
            
            % legParamsFileName
            if (~exist('legParamsFileName', 'var'))
                legParamsFileName = '';
            end
            
            % TODO Also add loading of ACA params as required?
            
            % quiet
            if (~exist('quiet', 'var'))
                quiet = 0;
            end
            
            % Set actuation parameters filename
            this.actParamsFileName = actParamsFileName;
            
            % Get leg object
            this.leg    = Leg_3DoF(legParamsFileName);
            
            % Get ACA objects and their remaining parameters
            this.a1     = ACA('ACA_test_params');
            this.a2     = ACA('ACA_test_params');
            this.a3     = ACA('ACA_test_params');
            
            % Set default pretension parameter
            this.p = [0.035 0.035 0.035];
 
                
            % For low-pass filters / exp. smoothing below
            Ts      = 1e-3;
            f_c     = 200; % [Hz]
            x       = (2 * pi * Ts * f_c);
            a_200   = x / (x+1);
            
            % Default control parameters
            this.control.Ts             = 1e-3;     % Timestep [s] (1 kHz)
            this.control.K_p_leg        = [500 500 500];    % Proportional gain for leg pos. ctrl.
            this.control.K_d_leg        = [60 60 60];       % Derivative gain for leg pos. ctrl.
            this.control.a_leg          = a_200;    % Derivative exp. smooth. factor
            this.control.limitCoP       = 0;        % Whether to try to keep the CoP within the foot by limiting ankle torque
            this.control.Fe_enable      = 0;        % Enable/disable external force disturbance (acting on trunk)
            this.control.Fe_mag         = [20; 0; 0]; % External force disturbance magnitude (x,y,tau)
            this.control.Fe_duration    = 0.2;      % External force disturbance duration [s]
            this.control.Fe_period      = 5.0;      % External force disturbance period [s]
            
            % If optimization result exists, load it to set actuation
            % topology and stiffnesses
            if (~strcmp(this.actParamsFileName, ''))
                % Load optimization results to set topology
                load(this.actParamsFileName, 'psi', 'T', 'actuatorParams');
                disp(['INFO: Loaded actuation topology from ' this.actParamsFileName]);
                % type is automagically set from topology
           
                this.a1.params.p_static     = actuatorParams{1}.p; %#ok<*USENS>
                this.a2.params.p_static     = actuatorParams{2}.p;
                this.a3.params.p_static     = actuatorParams{3}.p;
                this.a1.params.k_p          = actuatorParams{1}.k_p;
                this.a2.params.k_p          = actuatorParams{2}.k_p;
                this.a3.params.k_p          = actuatorParams{3}.k_p;
                
                % Give info on loaded topology
                if (quiet == 0)
                    if (strcmpi(actuatorParams{1}.type, 'mono'))
                        disp('INFO: Actuation topology is monoarticulated.');
                    elseif (strcmpi(actuatorParams{1}.type, 'bi'))
                        disp('INFO: Actuation topology is biarticulated.');
                    else
                        disp('INFO: Actuation topology is noESB.');
                    end
                end
                
                % Update ACA pretension limits based on the actuation
                % topology
                if (strcmpi(actuatorParams{1}.type, 'mono'))
                    % Monoarticulated
                    this.a1.params.p_range = this.leg.params.p_range_mono(1,:);
                    this.a2.params.p_range = this.leg.params.p_range_mono(2,:);
                    this.a3.params.p_range = this.leg.params.p_range_mono(3,:);
                elseif (strcmpi(actuatorParams{1}.type, 'bi'))
                    % Biarticulated
                    this.a1.params.p_range = this.leg.params.p_range_bi(1,:);
                    this.a2.params.p_range = this.leg.params.p_range_bi(2,:);
                    this.a3.params.p_range = this.leg.params.p_range_bi(3,:);
                end
                
            else
                % If not, set default actuation topology and use
                % default stiffnesses
                if (quiet == 0)
                    disp('WARN: No optimized topology specified, setting default.');
                    disp('INFO: Setting noESB topology.');
                end
                
                % No ESB
                T = zeros(this.leg.M, 3); % There are 3 actuated joints
            end

            % Set topologies
            this.a1.setTopology(T,1);
            this.a2.setTopology(T,2);
            this.a3.setTopology(T,3);
            
            % Default reference
            omega   = pi/4;
            l3      = this.leg.params.l3; % Hip segment
            this.ref.omega          = omega;        % Motion frequency
            this.ref.T              = 2*pi/omega;   % Motion period
            %this.ref.x_hip_amp     = 0.00 * l3;    % Hip x reference amplitude
            %this.ref.x_hip_bias    = 0.00 * l3;    % Hip x reference bias
            this.ref.x_hip_amp      = 0.30 * l3;    % Hip x reference amplitude
            this.ref.x_hip_bias     = 0.10 * l3;    % Hip x reference bias
            this.ref.x_hip_freq     = omega;        % Hip x reference frequency
            this.ref.x_hip_phase    = 0.0;          % Hip x reference phase
            %this.ref.y_hip_amp     = 0.48 * l3;    % Hip y reference amplitude
            %this.ref.y_hip_bias    = 1.50 * l3;    % Hip y reference bias
            this.ref.y_hip_amp      = 0.42 * l3;    % Hip y reference amplitude
            this.ref.y_hip_bias     = 1.53 * l3;    % Hip y reference bias
            this.ref.y_hip_freq     = omega;        % Hip y reference frequency
            this.ref.y_hip_phase    = pi/8;         % Hip y reference phase
            this.ref.use_random     = 0;            % Do not use random reference by default (use this.ref_use_random() to switch!)
    
            % Set initial leg, actuator and control states from reference at t=0
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            %[q_0, q_d_0]    = this.q_ref(0);
            %q_0(2)          = 0.02; % Leg starts slightly in the air
            %this.setInitialStates(q_0, q_d_0);
            
            % Set initial leg, actuator and control states from a squat position
            % Leg: q = [x1, y1, theta, q1, q2, q3]
%             q_0     = this.ref.random.q_ref ;       
%             q_d_0   = this.ref.random.q_d_ref;             
%             this.setInitialStates(q_0, q_d_0);


            % Set initial leg, actuator and control states from straight
            % leg configuration
            % Leg: q = [x1, y1, theta, q1, q2, q3]
%             q_0     = [0; 0.02; 0; 0.0; 0.0; 0.0]; % Leg starts slightly in the air
%             q_d_0   = [0; 0; 0; 0; 0; 0];
%             this.setInitialStates(q_0, q_d_0);
            
            %%%%%%%%%%%%%%%%
            
            % Set initial leg, actuator and control states
            % Use modified reference
            %q_0     = [0; 0.05; 0; -0.5; 0.8; -0.5];
            %q_d_0   = [0; 0; 0; 0; 0; 0];
            %this.setInitialStates(q_0, q_d_0);
            %[x_hip, y_hip, ~]       = this.leg.calc_fwdKin_named(q_0, 'hip');
            %this.ref.x_hip_amp      = 0;
            %this.ref.y_hip_amp      = 0.10;
            %this.ref.x_hip_bias     = 0.03;
            
            % Faster and high squat (possibly hopping)
            %[x_hip, y_hip, ~]       = this.leg.calc_fwdKin_named(q_0, 'hip');
            %this.ref.y_hip_bias     = y_hip - 0.12;
            %this.ref.x_hip_bias     = 0.06;
            %this.ref.x_hip_amp      = 0.05;
            %omega = 0.5*pi;
            %this.ref.x_hip_freq     = omega;
            %this.ref.y_hip_freq     = omega;
            
        end
        
        
        %__________________________________________________________________
        % Set initial states of leg and actuators properly
        function [ ] = setInitialStates( this, q_0, q_d_0 )
            % Set initial leg state
            this.leg.x0     = [q_0; q_d_0];
            
            % Get joints
            q_joints_0      = q_0(4:6);
            q_joints_d_0    = q_d_0(4:6);
            
            % Default control variables
            this.control.q_ref      = q_joints_0;
            this.control.q_d_ref    = q_joints_d_0;
            G                       = this.leg.calc_G(q_0);
            this.control.tau_ref    = -G(4:6);
            this.control.e_d_prev   = zeros(size(q_joints_0));

            % Set initial positions of PB and ESB rotors from initial leg
            % state, and set initial pretension references equal to the initial
            % positions.
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            % q_joints = [q1, q2, q3]
            this.a1.x0(5) = q_joints_0(1);  % PB rotor initial position equal to joint initial position
            this.a2.x0(5) = q_joints_0(2);  % PB rotor initial position equal to joint initial position
            this.a3.x0(5) = q_joints_0(3);  % PB rotor initial position equal to joint initial position
            this.a1.x0(6) = this.p(1);      % Pretension position
            this.a2.x0(6) = this.p(2);      % Pretension position
            this.a3.x0(6) = this.p(3);      % Pretension position
            this.a1.x0(4) = this.p(1) + this.a1.topology.t * q_joints_0; % dL_p = p +t * q_0
            this.a2.x0(4) = this.p(2) + this.a2.topology.t * q_joints_0; % dL_p = p +t * q_0
            this.a3.x0(4) = this.p(3) + this.a3.topology.t * q_joints_0; % dL_p = p +t * q_0
            this.a1.control.p_opt_prev = this.a1.x0(6);     % Pretension opt. ref. filter state
            this.a2.control.p_opt_prev = this.a2.x0(6);     % Pretension opt. ref. filter state
            this.a3.control.p_opt_prev = this.a3.x0(6);     % Pretension opt. ref. filter state
            this.a1.control.p_ref_prev = this.a1.x0(6);     % Pretension reference state
            this.a2.control.p_ref_prev = this.a2.x0(6);     % Pretension reference state
            this.a3.control.p_ref_prev = this.a3.x0(6);     % Pretension reference state
        end
        
        
        %__________________________________________________________________
        % State evolution - function to be handed to ode45()
        function [ dx ] = dx_ode( this, t, x )
            [dx, ~, ~, ~] = this.dx(t,x);
        end

        
        %__________________________________________________________________
        % State evolution
        function [ dx, y, u, tau_ref ] = dx( this, t, x )

            % x = [x_a1; x_a2; x_a3; x_leg]
            % u = [u_a1; u_a2; u_a3; u_leg]

            % Get states

            % Get the ACA and mass states
            stateIdx_a1     = 1 : this.a1.N;
            stateIdx_a2     = max(stateIdx_a1)+1 : max(stateIdx_a1)+this.a2.N;
            stateIdx_a3     = max(stateIdx_a2)+1 : max(stateIdx_a2)+this.a3.N;
            stateIdx_leg    = max(stateIdx_a3)+1 : max(stateIdx_a3)+this.leg.N;
            x_a1            = x(stateIdx_a1);
            x_a2            = x(stateIdx_a2);
            x_a3            = x(stateIdx_a3);
            x_leg           = x(stateIdx_leg);


            % Get outputs

            % Connect the leg velocities q_d to the third inputs of the ACAs to
            % obtain their outputs, as there is direct pass-through.
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            % Leg: x = [q; q_d]
            q_leg   = x_leg(1 : this.leg.N/2); %#ok<*NASGU>
            q_leg_d = x_leg(this.leg.N/2 + 1 : this.leg.N);
            
            % Get only the joint states for the actuators
            q_joints    = q_leg(4:6);
            q_joints_d  = q_leg_d(4:6);

            % Define inputs for direct feed-through
            u_a1    = [0; 0; q_joints_d];  % u_ax = [i_1; i_2; [q_joints_d]]
            u_a2    = [0; 0; q_joints_d];  % ACA outputs depend on 3rd input (direct feed-through) due to friction
            u_a3    = [0; 0; q_joints_d];  % We can leave i_1, i_2 zero for now

            % Get the ACA outputs, to calculate tau_p, which is used for
            % the controllers below
            y_a1        = this.a1.y(t, x_a1, u_a1);
            y_a2        = this.a2.y(t, x_a2, u_a2);
            y_a3        = this.a3.y(t, x_a3, u_a3);
            
            % ESB torques
            tau_p_1     = y_a1(this.a1.outputIdx_tau_p);
            tau_p_2     = y_a2(this.a2.outputIdx_tau_p);
            tau_p_3     = y_a3(this.a3.outputIdx_tau_p);
            tau_p       = tau_p_1 + tau_p_2 + tau_p_3; % Net ESB torque
            tau_s_1     = tau_p_2 + tau_p_3;
            tau_s_2     = tau_p_1 + tau_p_3;
            tau_s_3     = tau_p_1 + tau_p_2;

            % Output torques
            tau_a1      = y_a1(this.a1.outputIdx_tau);
            tau_a2      = y_a2(this.a2.outputIdx_tau);
            tau_a3      = y_a3(this.a3.outputIdx_tau);
            
            % Calculate the leg input from the combined output torques
            %u_leg = [tau_1; tau_2; tau_3]
            tau_leg = tau_a1 + tau_a2 + tau_a3;
            u_leg   = tau_leg;

            % Calculate/get new inputs

            % Calculate(update) joint controllers
            tau_ref = this.control.tau_ref; % Output
            if (mod(t, this.control.Ts) == 0) % At control timesteps..
                % Get reference
                if (this.ref.use_random == 1)
                    [q_ref, q_d_ref] = this.q_ref_random(t);
                else
                    [q_ref, q_d_ref] = this.q_ref(t);
                end
                
                % Get joint references
                q_joints_ref    = q_ref(4:6);
                q_joints_d_ref  = q_d_ref(4:6);
                
                % Leg control (PD with gravity compensation)
                e           = q_joints_ref - q_joints;
                e_d         = q_joints_d_ref - q_joints_d;
                e_d_prev    = this.control.e_d_prev;
                a_leg       = this.control.a_leg;
                e_d         = a_leg * e_d + (1 - a_leg) * e_d_prev;
                G           = this.leg.calc_G(q_leg);
                tau_ref     =   -G(4:6) + ...
                                this.control.K_p_leg' .* e + ...
                                this.control.K_d_leg' .* e_d;
                            
                % Limit ankle torque based on CoP
                % We use Fry from the static calculation, and the real
                % ankle torque. So this is approximate.
                if (this.control.limitCoP == 1)
                    % Define the CoP bounds based on the foot size and the
                    % current position of the floating base of the leg
                    CoP_min = q_leg(1) - this.leg.params.r_heel + 0.01;
                    CoP_max = q_leg(1) + this.leg.params.r_toe - 0.01;
                    [ Fry, ~ ] = this.leg.calc_GRF_CoP_static(q_leg);
                    CoP = tau_leg(1) / Fry; % Ankle joint
                    if (CoP < CoP_min)
                        tau_ref(1) = CoP_min * Fry;
                    elseif (CoP > CoP_max)
                        tau_ref(1) = CoP_max * Fry;
                    end
                end
                
                % Set resulting values into control struct
                this.control.q_ref      = q_ref;
                this.control.q_d_ref    = q_d_ref;
                this.control.tau_ref    = tau_ref;
                this.control.e_d_prev   = e_d;
                
                % ACA controllers (result used below)
                [~, ~] = this.a1.controller(t, x_a1, tau_ref(1), q_joints, q_joints_d, tau_p, tau_s_1);
                [~, ~] = this.a2.controller(t, x_a2, tau_ref(2), q_joints, q_joints_d, tau_p, tau_s_2);
                [~, ~] = this.a3.controller(t, x_a3, tau_ref(3), q_joints, q_joints_d, tau_p, tau_s_3);
            end;

            % Build ACA inputs
            % Get input currents from ACA controllers
            u_a1    = [this.a1.control.i_1; this.a1.control.i_2; q_joints_d];
            u_a2    = [this.a2.control.i_1; this.a2.control.i_2; q_joints_d];
            u_a3    = [this.a3.control.i_1; this.a3.control.i_2; q_joints_d];

            % Build the complete input vector
            u       = [u_a1; u_a2; u_a3; u_leg];
            
            
            % Get dynamics

            % Get actuator dynamics
            dx_a1   = this.a1.dx(t, x_a1, u_a1);
            dx_a2   = this.a2.dx(t, x_a2, u_a2);
            dx_a3   = this.a3.dx(t, x_a3, u_a3);

            % Build ground reaction force (GRF)
            % The floor is a spring-damper in the y-direction, and
            % viscous+Coulomb friction in the x-direction.
            % F_GRF = [F_heel_x; F_heel_y; F_toe_x; F_toe_y]
            [~, y_heel, ~]              = this.leg.calc_fwdKin_named(q_leg, 'heel');
            [~, y_toe, ~]               = this.leg.calc_fwdKin_named(q_leg, 'toe');
            [x_ankle_d, y_ankle_d, ~]   = this.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'heel');
            [x_toe_d, y_toe_d, ~]       = this.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'toe');
            F_ankle_x   = 0;
            F_ankle_y   = 0;
            F_toe_x     = 0;
            F_toe_y     = 0;
            floor_K     = this.leg.params.floor_K;
            floor_D     = this.leg.params.floor_D;
            if (y_heel <= 0)
                F_ankle_y   = floor_K * (0.0 - y_heel) + max(0, floor_D * (0.0 - y_ankle_d));
                coulomb     = -this.leg.params.coulomb * tanh(500*x_ankle_d);
                F_ankle_x   = coulomb + F_ankle_y * this.leg.params.mu * (0.0 - x_ankle_d);
            end
            if (y_toe <= 0)
                F_toe_y     = floor_K * (0.0 - y_toe) + max(0, floor_D * (0.0 - y_toe_d));
                coulomb     = -this.leg.params.coulomb * tanh(500*x_toe_d);
                F_toe_x     = coulomb + F_toe_y * this.leg.params.mu * (0.0 - x_toe_d);
            end
            F_GRF = [F_ankle_x; F_ankle_y; F_toe_x; F_toe_y];
          
            
            % Build external interaction force
            % Fe = [Fe_x; Fe_y; Fe_tau]
            if (    this.control.Fe_enable == 1 && ...
                    mod(t,this.control.Fe_period) >= 0 && ...
                    mod(t,this.control.Fe_period) < this.control.Fe_duration    )
                Fe = this.control.Fe_mag;
            else
                Fe = [0; 0; 0];
            end
            
            % Get leg dynamics
            dx_leg  = this.leg.dx(t, x_leg, u_leg, F_GRF, Fe);

            % Assemble state derivatives
            dx      = [dx_a1; dx_a2; dx_a3; dx_leg];
            
            
            % (Re-)calculate outputs
            
            % Re-calculate the ACA outputs now that their inputs are fully known
            y_a1    = this.a1.y(t, x_a1, u_a1);
            y_a2    = this.a2.y(t, x_a2, u_a2);
            y_a3    = this.a3.y(t, x_a3, u_a3);
            
            % Get the leg output
            y_leg   = this.leg.y(t, x_leg);
            
            % Get the total output
            y       = [y_a1; y_a2; y_a3; y_leg];

        end

        
        %__________________________________________________________________
        % Calculate the references as a function of time
        function [ q_ref, q_d_ref ] = q_ref( this, t )
  
            % Calculate references for joints
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            q_ref       = zeros(6,length(t));
            q_ref(1,:)  = 0;
            q_ref(2,:)  = 0;
            q_ref(3,:)  = 0;
            
            % Timing stages, time span in seconds
            stg1 = 0.1; stg2 = 0.3; stg3 = 0.4;

            % Stage 1
            if (0<=t) & (t<=stg1)                    
                    q_ref(4,:)  = -0.5;
                    q_ref(5,:)  = 1.6;
                    q_ref(6,:)  = -1.2;
            end

             % Stage 2: Hip, knee and ankle extension
            if (stg1<t) & (t<=(stg1+stg2))
                    q_ref(4,:)  = -0.7 ; 
                    q_ref(5,:)  = 1.5 ;
                    q_ref(6,:)  = -1.2 ;
            end    
            
            
           % Stage 3:  push off
            if (stg1+stg2<t) & (t<=(stg1+stg2+stg3))
                    q_ref(4,:)  = 0.8;
                    q_ref(5,:)  = 0.6;
                    q_ref(6,:)  = -0.2;                      
            end  

          % Stage 4: Fly
            if (stg1+stg2+stg3<t) 
                    q_ref(4,:)  = -0.4;
                    q_ref(5,:)  = 0.2;
                    q_ref(6,:)  = 0;                        
            end  
                   
            % Calculate reference velocities                                     
            %q_d_ref = diff(q_ref')' / this.control.Ts;
            %q_d_ref = [q_d_ref(:,1), q_d_ref]; % Approximate first entry by extrapolation
            q_d_ref = zeros(6,length(t));
            
            
        end
        
        %__________________________________________________________________
        % Return random references as a function of time
        function [ q_ref, q_d_ref ] = q_ref_random( this, t )
            idx         = round(t / this.control.Ts)+1;
            q_ref       = this.ref.random.q_ref(:,idx);
            q_d_ref     = this.ref.random.q_d_ref(:,idx);
        end

        %__________________________________________________________________
        % Generate motion reference by spline interpolation of random points
        function [ t, q_ref, q_d_ref ] = q_ref_generate_spline( this, tspan, dt )
            % Check tspan
            if (length(tspan) ~= 2 || tspan(2)<=tspan(1) || tspan(1) < 0)
                error('Invalid tspan!');
            end
            % Check dt
            if (dt < 0) % This might need more checks
                error('Invalid dt!');
            end
            
            % Generate random points every dt
            n           = floor(tspan(2)/dt) + 1;
            p           = this.q_ref_generate_points(n);
            t_p         = dt * (0:n-1);
            t           = 0:this.control.Ts:tspan(2);

            % If we have only one point just repeat it
            if (n == 1)
                x_hip       = repmat(p(1), [1 length(t)]);
                y_hip       = repmat(p(2), [1 length(t)]);
            else
                % Smooth out Cartesian reference
                x_hip       = interp1(t_p, p(1,:), t, 'spline')';
                y_hip       = interp1(t_p, p(2,:), t, 'spline')';
            end

            % Calculate reference positions

            % Calculate current desired beta, r as a function of time
            beta    = atan2(-x_hip, y_hip);
            r       = sqrt(x_hip.^2 + y_hip.^2);

            % Calculate references for joints
            % This assumes l2 == l3 (shank/hip equal length)
            q_ref(1,:)  = zeros(1,length(t));
            q_ref(2,:)  = zeros(1,length(t));
            q_ref(3,:)  = zeros(1,length(t));
            q_ref(4,:)  = -acos(r / (this.leg.params.l2 + this.leg.params.l3)) + beta;
            q_ref(5,:)  = 2 * acos(r / (this.leg.params.l2 + this.leg.params.l3));
            q_ref(6,:)  = -(q_ref(4,:) + q_ref(5,:));
            
            % Obtain static CoP from the leg model
            % Obtaining the dynamic CoP here would require inverse dynamics
            [ ~, CoP_static ] = this.leg.calc_GRF_CoP_static(q_ref);
            % Correct CoP_static to be w.r.t. the ankle joint
            CoP_static = CoP_static - (q_ref(1,:) - this.leg.params.r1);
            
            % Show generated x-y hip trajectory
            figure; grid on;
            plot(x_hip, y_hip);
            xlabel('x_{hip}'); ylabel('y_{hip}');
            title('x-y hip trajectory (w.r.t. ankle joint!)');
            
            % Show beta, r, CoP over time
            figure; grid on; hold on;
            plot(t, beta, 'r');
            plot(t, r, 'b');
            plot(t, CoP_static, 'Color', [0 0.7 0]);
            plot([0 max(t)], [-this.leg.params.r_heel -this.leg.params.r_heel], '--', 'Color', [0 0.7 0]);
            plot([0 max(t)], [this.leg.params.r_toe this.leg.params.r_toe], '--', 'Color', [0 0.7 0]);
            legend('beta', 'r', 'static CoP', 'Location', 'best');
            xlabel('t');
            title('beta, r (w.r.t. ankle joint!)');

            % Check if trajectory is within reach
            if (find(r > this.leg.params.l2 + this.leg.params.l3))                
                % Abort with error
                error('Infeasible trajectory: Out of reach');
            end
            
            % Get joint limits
            q_limits = this.leg.params.q_limits;
            
            % Show random reference against joint limits
            figure; grid on; hold on;
            s = fill([0 max(t) max(t) 0], [q_limits(1,2) q_limits(1,2) q_limits(1,1) q_limits(1,1)], [0.9 0.9 1.0]);
            s.LineStyle = 'none';
            s = fill([0 max(t) max(t) 0], [q_limits(2,2) q_limits(2,2) q_limits(2,1) q_limits(2,1)], [1.0 0.9 0.9]);
            s.LineStyle = 'none';
            %s = fill([0 max(t) max(t) 0], [q_limits(3,2) q_limits(3,2) q_limits(3,1) q_limits(3,1)], [0.9 1.0 0.9]);
            %s.LineStyle = 'none';
            h1 = plot(t, q_ref(4,:), 'Color', [0 0 1]);
            h2 = plot(t, q_ref(5,:), 'Color', [1 0 0]);
            h3 = plot(t, q_ref(6,:), 'Color', [0 0.7 0]);
            plot([0 max(t)], [q_limits(1,1) q_limits(1,1)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(1,2) q_limits(1,2)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(2,1) q_limits(2,1)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(2,2) q_limits(2,2)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(3,1) q_limits(3,1)], '--', 'Color', [0 0.8 0]);
            plot([0 max(t)], [q_limits(3,2) q_limits(3,2)], '--', 'Color', [0 0.8 0]);
            legend([h1 h2 h3], 'q_1', 'q_2', 'q_3');
            title('Random reference vs joint limits');
            drawnow;
            
            % Check trajectory against joint limits
            if (    ~isempty(find(q_ref(4,:) < q_limits(1,1), 1)) || ... % q1 LB
                    ~isempty(find(q_ref(4,:) > q_limits(1,2), 1)) || ... % q1 UB
                    ~isempty(find(q_ref(5,:) < q_limits(2,1), 1)) || ... % q2 LB
                    ~isempty(find(q_ref(5,:) > q_limits(2,2), 1)) || ... % q2 UB
                    ~isempty(find(q_ref(6,:) < q_limits(3,1), 1)) || ... % q3 LB
                    ~isempty(find(q_ref(6,:) > q_limits(3,2), 1)) )      % q3 UB
                % Throw error
                error('Infeasible trajectory: Outside joint limits');
            end

            % Calculate joint velocities
            q_d_ref = diff(q_ref')' / this.control.Ts;
            q_d_ref = [q_d_ref(:,1), q_d_ref]; % Approximate first entry by extrapolation
        end
        
        %__________________________________________________________________
        % Generate points for complex random references
        function [ p ] = q_ref_generate_points( this, n )
            % Generate random points
            % Set seed based on current time, including subseconds
            rng('shuffle');
            p = rand([2,n]);

            % If we have more than one point...
            if (size(p,2) > 1)
                % Get point ranges
                xRange = max(p(1,:)) - min(p(1,:));
                yRange = max(p(2,:)) - min(p(2,:));

                % Scale ranges
                xRange_new = 0.35;%0.4;%0.3;
                yRange_new = 0.70;%0.5;%0.4;
                p = [   xRange_new * this.leg.params.l2 * p(1,:) / xRange; ...
                        yRange_new * this.leg.params.l2 * p(2,:) / yRange       ];

                % Move means
                xMean_new = 0.05 * this.leg.params.l2;
                yMean_new = 1.55 * this.leg.params.l2;
                p = [   p(1,:) + (xMean_new - mean(p(1,:))); ...
                        p(2,:) + (yMean_new - mean(p(2,:)))          ];
                    
            else
                error('Rewrite this');
                % For one point, just move the distribution
                p = 0.3 * this.leg.params.l2 * p;
                p = [   p(1,:); ...
                        p(2,:) + ((1.45-0.20) * this.leg.params.l2)     ];
            end
        end
        
        %__________________________________________________________________
        % Generate motion reference by spline interpolation of random points
        % in the q1,q2 space, while taking the CoP into account
        function [ t, q_ref, q_d_ref ] = q_ref_generate_random( this, tspan, dt )
            % Check tspan
            if (length(tspan) ~= 2 || tspan(2)<=tspan(1) || tspan(1) < 0)
                error('Invalid tspan!');
            end
            % Check dt
            if (dt < 0) % This might need more checks
                error('Invalid dt!');
            end
            
            % Generate random points every dt, so:
            n           = floor(tspan(2)/dt) + 1;       % Number of points to generate
            t_p         = dt * (0:n-1);                 % Times associated with the points
            t           = 0:this.control.Ts:tspan(2);   % Time vector
            CoP_margin  = 0.02;                         % CoP margin w.r.t. support polygon boundary
            
            % Set RNG seed based on current time, including subseconds
            rng('shuffle');
            
            % Get joint limits
            q_limits = this.leg.params.q_limits;
            
            trajectoryValid = 0;
            while (trajectoryValid == 0)
                % Generate n random points in joint space
                % For each point we check whether it is valid, i.e. the CoP
                % lies within the support polygon.
                p_q = zeros(this.leg.N/2, n);
                for i=1:n
                    validPoint = 0;
                    while (validPoint == 0)
                        validRadius = 0;
                        validCoP = 0;
                        
                        % Generate a random point in the joint space within the
                        % joint limits, and set q_3 s.t. the trunk is vertical.
                        q_1_range = q_limits(1,2) - q_limits(1,1);
                        q_2_range = q_limits(2,2) - q_limits(2,1);
                        q_1 = q_limits(1,1) + q_1_range * rand();
                        q_2 = q_limits(2,1) + q_2_range * rand();
                        q_3 = -(q_1 + q_2);

                        % Generate the full configuration vector
                        q = [0; 0; 0; q_1; q_2; q_3];
                        
                        % Convert the points in generalised coordinates to Cartesian
                        % coordinates of the hip, and smooth out the reference.
                        [ x_hip, y_hip, ~ ] = this.leg.calc_fwdKin_named(q, 'hip');
                        
                        % Calculate hip x-position w.r.t. the ankle joint for inverse
                        % kinematics
                        x_hip_ankle = x_hip + this.leg.params.r1;
                        
                        % Calculate polar coordinates beta, r w.r.t. ankle
                        beta    = atan2(-x_hip_ankle, y_hip);
                        r       = sqrt(x_hip_ankle.^2 + y_hip.^2);
                        
                        % Check if the polar radius is sufficiently small
                        % This is disabled for now. The radius is still
                        % checked after the spline interpolation however,
                        % so there is no chance of the final trajectory
                        % being unreachable.
                        %if (r <= 0.95 * (this.leg.params.l2 + this.leg.params.l3))
                        %    validRadius = 1;
                        %else
                        %    disp(['WARN: Ignoring invalid point [q_1; q_2; q_3] = [' sprintf('%5.2f', q_1) '; ' sprintf('%5.2f', q_2) '; ' sprintf('%5.2f', q_3) '] with polar radius ' sprintf('%6.3f', r) '..']);
                        %end
                        validRadius = 1;
                        
                        % Obtain the static CoP from the leg model.
                        % Obtaining the dynamic CoP would require inverse dynamics!
                        [ ~, CoP_static ] = this.leg.calc_GRF_CoP_static(q);

                        % Check if the CoP lies within the support polygon. It
                        % is defined w.r.t. the floating base (in the foot
                        % CoM), so we check as follows.
                        % We use twice the CoP margin here since the spline
                        % interpolation will exceed the points somewhat.
                        if (    CoP_static >= -this.leg.params.r_heel + 2*CoP_margin  && ...
                                CoP_static <= this.leg.params.r_toe - 2*CoP_margin    )
                            validCoP = 1;
                        else
                            %disp(['WARN: Ignoring invalid point [q_1; q_2; q_3] = [' sprintf('%5.2f', q_1) '; ' sprintf('%5.2f', q_2) '; ' sprintf('%5.2f', q_3) '] with CoP ' sprintf('%6.3f', CoP_static) '..']);
                        end
                        
                        % If all conditions are met, we can add this point.
                        if (validRadius && validCoP)
                            %disp(['Adding point [q_1; q_2; q_3] = [' sprintf('%5.2f', q_1) '; ' sprintf('%5.2f', q_2) '; ' sprintf('%5.2f', q_3) '] with polar radius ' sprintf('%6.3f', r) ' and CoP ' sprintf('%6.3f', CoP_static) '..']);
                            validPoint = 1;
                        end
                    end

                    % We now have a valid point in generalised coordinates, add it.
                    p_q(:,i) = [0; 0; 0; q_1; q_2; q_3];
                end

                % Convert the points in generalised coordinates to Cartesian
                % coordinates of the hip, and smooth out the reference.
                [ x_hip, y_hip, ~ ] = this.leg.calc_fwdKin_named(p_q, 'hip');

                % If we have only one point just repeat it
                if (n == 1)
                    x_hip       = repmat(x_hip, [1 length(t)]);
                    y_hip       = repmat(y_hip, [1 length(t)]);
                else
                    % Smooth out Cartesian reference
                    x_hip       = interp1(t_p, x_hip, t, 'spline')';
                    y_hip       = interp1(t_p, y_hip, t, 'spline')';
                end
                
                % Calculate hip x-position w.r.t. the ankle joint for inverse
                % kinematics
                x_hip_ankle = x_hip + this.leg.params.r1;

                % Calculate references in generalised coordinates

                % Calculate polar coordinates beta, r over time
                beta    = atan2(-x_hip_ankle, y_hip);
                r       = sqrt(x_hip_ankle.^2 + y_hip.^2);
                
                % Check if trajectory is within reach
                if (find(r > this.leg.params.l2 + this.leg.params.l3))                
                    disp('WARN: Infeasible trajectory: Out of reach. Trying again..');
                    
                    % Debug
                    %figure(1); clf; hold on;
                    %[ x_hip_p, y_hip_p, ~ ] = this.leg.calc_fwdKin_named(p_q, 'hip');
                    %r_p = sqrt(x_hip_p.^2 + y_hip_p.^2);
                    %scatter(linspace(1,length(t),length(r_p)), r_p);
                    %drawnow;
                    
                    continue;
                end

                % Calculate references for joints
                % This assumes l2 == l3 (shank/hip equal length)
                q_ref(1,:)  = zeros(1,length(t));
                q_ref(2,:)  = zeros(1,length(t));
                q_ref(3,:)  = zeros(1,length(t));
                q_ref(4,:)  = -acos(r / (this.leg.params.l2 + this.leg.params.l3)) + beta;
                q_ref(5,:)  = 2 * acos(r / (this.leg.params.l2 + this.leg.params.l3));
                q_ref(6,:)  = -(q_ref(4,:) + q_ref(5,:));

                % Obtain static CoP from the leg model
                % Obtaining the dynamic CoP here would require inverse dynamics
                [ ~, CoP_static ] = this.leg.calc_GRF_CoP_static(q_ref);
                % Correct CoP_static to be w.r.t. the floating base
                % This is not required because x1==0 (see above)so the global
                % frame and the floating base frame coincide.
                %CoP_static = CoP_static - (q_ref(1,:);

                % Check if the CoP is within the support polygon
                if (    ~isempty(find(CoP_static < -this.leg.params.r_heel + CoP_margin, 1)) || ...
                        ~isempty(find(CoP_static > this.leg.params.r_toe - CoP_margin, 1))   )
                    disp('WARN: Infeasible trajectory: CoP not within support polygon margins. Trying again..');
                    
                    % Debug plot
                    [ ~, CoP_static_p ] = this.leg.calc_GRF_CoP_static(p_q);
                    figure(2); clf; hold on;
                    plot(CoP_static);
                    plot([0 length(CoP_static)], [-this.leg.params.r_heel + CoP_margin -this.leg.params.r_heel + CoP_margin], '--');
                    plot([0 length(CoP_static)], [this.leg.params.r_toe - CoP_margin this.leg.params.r_toe - CoP_margin], '--');
                    plot([0 length(CoP_static)], [-this.leg.params.r_heel -this.leg.params.r_heel]);
                    plot([0 length(CoP_static)], [this.leg.params.r_toe this.leg.params.r_toe]);
                    scatter(linspace(1,length(t),length(CoP_static_p)), CoP_static_p);
                    ylim([-this.leg.params.r_heel - 0.05, this.leg.params.r_toe + 0.05]);
                    drawnow;
                    
                    continue;
                end

                % Check trajectory against joint limits
                if (    ~isempty(find(q_ref(4,:) < q_limits(1,1), 1)) || ... % q1 LB
                        ~isempty(find(q_ref(4,:) > q_limits(1,2), 1)) || ... % q1 UB
                        ~isempty(find(q_ref(5,:) < q_limits(2,1), 1)) || ... % q2 LB
                        ~isempty(find(q_ref(5,:) > q_limits(2,2), 1)) || ... % q2 UB
                        ~isempty(find(q_ref(6,:) < q_limits(3,1), 1)) || ... % q3 LB
                        ~isempty(find(q_ref(6,:) > q_limits(3,2), 1)) )      % q3 UB
                    disp('WARN: Infeasible trajectory: Outside joint limits. Trying again..');
                    continue;
                end
                
                % If we reach this we can carry on because the trajectory
                % is valid.
                disp('Success! Feasible random trajectory found.');
                trajectoryValid = 1;
            end
            
            % Show generated x-y hip trajectory
            figure; grid on; hold on;
            plot(x_hip, y_hip);
            plot([-this.leg.params.r_heel -this.leg.params.r_heel], [min(y_hip) max(y_hip)], '--');
            plot([this.leg.params.r_toe this.leg.params.r_toe], [min(y_hip) max(y_hip)], '--');
            xlabel('x_{hip}'); ylabel('y_{hip}');
            title('x-y hip trajectory');
            
            % Show beta, r, CoP over time
            figure; grid on; hold on;
            plot(t, beta, 'r');
            plot(t, r, 'b');
            plot(t, CoP_static, 'Color', [0 0.7 0]);
            plot([0 max(t)], [-this.leg.params.r_heel -this.leg.params.r_heel], '--', 'Color', [0 0.7 0]);
            plot([0 max(t)], [this.leg.params.r_toe this.leg.params.r_toe], '--', 'Color', [0 0.7 0]);
            legend('beta', 'r', 'static CoP', 'Location', 'best');
            xlabel('t');
            title('beta, r (w.r.t. ankle joint!), and CoP in floating base frame');
            
            % Show random reference against joint limits
            figure; grid on; hold on;
            s = fill([0 max(t) max(t) 0], [q_limits(1,2) q_limits(1,2) q_limits(1,1) q_limits(1,1)], [0.9 0.9 1.0]);
            s.LineStyle = 'none';
            s = fill([0 max(t) max(t) 0], [q_limits(2,2) q_limits(2,2) q_limits(2,1) q_limits(2,1)], [1.0 0.9 0.9]);
            s.LineStyle = 'none';
            %s = fill([0 max(t) max(t) 0], [q_limits(3,2) q_limits(3,2) q_limits(3,1) q_limits(3,1)], [0.9 1.0 0.9]);
            %s.LineStyle = 'none';
            h1 = plot(t, q_ref(4,:), 'Color', [0 0 1]);
            h2 = plot(t, q_ref(5,:), 'Color', [1 0 0]);
            h3 = plot(t, q_ref(6,:), 'Color', [0 0.7 0]);
            plot([0 max(t)], [q_limits(1,1) q_limits(1,1)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(1,2) q_limits(1,2)], '--', 'Color', [0 0 0.8]);
            plot([0 max(t)], [q_limits(2,1) q_limits(2,1)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(2,2) q_limits(2,2)], '--', 'Color', [0.8 0 0]);
            plot([0 max(t)], [q_limits(3,1) q_limits(3,1)], '--', 'Color', [0 0.8 0]);
            plot([0 max(t)], [q_limits(3,2) q_limits(3,2)], '--', 'Color', [0 0.8 0]);
            legend([h1 h2 h3], 'q_1', 'q_2', 'q_3');
            title('Random reference vs joint limits');
            drawnow;

            % Calculate joint velocities
            q_d_ref = diff(q_ref')' / this.control.Ts;
            q_d_ref = [q_d_ref(:,1), q_d_ref]; % Approximate first entry by extrapolation
        end
        
        %__________________________________________________________________
        % Setup for using random reference
        function [ t, q_ref, q_d_ref ] = q_ref_jump( this, tspan, dt )
            % Check tspan
            if (length(tspan) ~= 2 || tspan(2)<=tspan(1) || tspan(1) < 0)
                error('Invalid tspan!');
            end
            
            % Generate random points every dt, so:
            n           = floor(tspan(2)/dt) + 1;       % Number of points to generate
            t_p         = dt * (0:n-1);                 % Times associated with the points
            t           = 0:this.control.Ts:tspan(2);   % Time vector
          
            % Get joint limits
            q_limits = this.leg.params.q_limits;
            
            % Generate x-y trajectory reference of the hip
            % Time instances when things happen
            t0 = 0;     % Start (always zero) and start lowering
            t1 = 3;     % End lowering
            t2 = 3.5;   % End wait for balance
            t3 = 4.1;   % End finish jump
            t4 = 4.9;   % End retracting
            t5 = 8.0;   % End standing up
            % Preallocate Cartesian hip coordinates
            x_hip_ankle     = zeros(1, n);
            y_hip           = zeros(1, n);
            for i=1:n
                ti = t_p(i);
                
                if (ti < t1)        % Lowering
                    x_hip_ankle(i)  = ((ti - t0)/(t1 - t0)) * 0.01;
                    y_hip(i)        = 0.60 - ((ti - t0)/(t1 - t0)) * 0.25;
                elseif (ti < t2)    % Waiting
                    x_hip_ankle(i)  = 0.01;
                    y_hip(i)        = 0.35;
                elseif (ti < t3)    % Jumping
                    x_hip_ankle(i)  = 0.01 + ((ti - t2)/(t3 - t2)) * 0.12;
                    y_hip(i)        = 0.35 + ((ti - t2)/(t3 - t2)) * 0.335;
                elseif (ti < t4)    % Retracting
                    x_hip_ankle(i)  = 0.13 - ((ti - t3)/(t4 - t3)) * 0.12;
                    y_hip(i)        = 0.685 - ((ti - t3)/(t4 - t3)) * 0.285;
                elseif (ti < t5)    % Standing up
                    x_hip_ankle(i)  = 0.01;
                    y_hip(i)        = 0.40 + ((ti - t4)/(t5 - t4)) * 0.25;
                else                % Static
                    x_hip_ankle(i)  = 0.01;
                    y_hip(i)        = 0.65;
                end
            end
            
            % Smooth out Cartesian reference
            x_hip_ankle     = interp1(t_p, x_hip_ankle, t, 'spline')';
            y_hip           = interp1(t_p, y_hip, t, 'spline')';
            
            % Calculate polar coordinates beta, r over time
            beta    = atan2(-x_hip_ankle, y_hip);
            r       = sqrt(x_hip_ankle.^2 + y_hip.^2);
            
            % Generate an ankle extension profile
            ankleExtension = zeros(1,n);
            % Time instances when things happen
            t1 = 3.90;  % Start ankle extension
            t2 = 4.15;  % End ankle extension
            t3 = 4.4;   % End ankle flexion
            for i=1:n
                ti = t_p(i);
                
                if (ti < t1)
                    ankleExtension(i) = 0;
                elseif (ti < t2)
                    ankleExtension(i) = ((ti - t1)/(t2 - t1)) * 0.4;
                elseif (ti < t3)
                    ankleExtension(i) = 0.4 - ((ti - t2)/(t3 - t2)) * 0.4;
                else
                    ankleExtension(i) = 0;
                end
            end
            ankleExtension = interp1(t_p, ankleExtension, t, 'spline');
            
            % Calculate references for joints
            % This assumes l2 == l3 (shank/hip equal length)
            q_ref(1,:)  = zeros(1,length(t));
            q_ref(2,:)  = zeros(1,length(t));
            q_ref(3,:)  = zeros(1,length(t));
            q_ref(4,:)  = -acos(r / (this.leg.params.l2 + this.leg.params.l3)) + beta;
            q_ref(5,:)  = 2 * acos(r / (this.leg.params.l2 + this.leg.params.l3));
            q_ref(6,:)  = -(q_ref(4,:) + q_ref(5,:)) - 0.08; % Trunk lean for jump
            
            % Add ankle extension afterwards (so that we don't disturb the hip angle)
            q_ref(4,:)  = q_ref(4,:) + ankleExtension;
            
            % Check trajectory against joint limits
            if (    ~isempty(find(q_ref(4,:) < q_limits(1,1), 1)) || ... % q1 LB
                    ~isempty(find(q_ref(4,:) > q_limits(1,2), 1)) || ... % q1 UB
                    ~isempty(find(q_ref(5,:) < q_limits(2,1), 1)) || ... % q2 LB
                    ~isempty(find(q_ref(5,:) > q_limits(2,2), 1)) || ... % q2 UB
                    ~isempty(find(q_ref(6,:) < q_limits(3,1), 1)) || ... % q3 LB
                    ~isempty(find(q_ref(6,:) > q_limits(3,2), 1)) )      % q3 UB
                error('Infeasible trajectory: Outside joint limits.');
            end
            
            % Check trajectory against imaginary parts (due to polar
            % conversion above)
            if (norm(imag(q_ref)) > 0)
               error('Infeasible trajectory: Imaginary components.');
            end
            
            % Calculate joint velocities
            q_d_ref = diff(q_ref')' / this.control.Ts;
            q_d_ref = [q_d_ref(:,1), q_d_ref]; % Approximate first entry by extrapolation
        end
        
        %__________________________________________________________________
        % Setup for using random reference
        function [ ] = ref_use_random( this, use_random, tspan, dt )
            if (use_random == 1)
                % Generate random reference
                % We can use multiple functions here:
                % - this.q_ref_generate_spline(tspan, dt);
                %   Generates splines based on random points in the x-y space
                % - this.q_ref_generate_random(tspan, dt);
                %   Generates splines based on random points in the q1,q2
                %   space, and takes the CoP into account.
                % - this.q_ref_jump(tspan, dt);
                %   Generates a jumping motion
                [   this.ref.random.t, ...
                    this.ref.random.q_ref, ...
                    this.ref.random.q_d_ref     ] = this.q_ref_jump(tspan, dt);
            
                % The below often does not work since we require the
                % elongation of the ESB bungees to be zero at t=0 and the
                % corresponding pretension positions are outside the
                % pretension position limits.
                % So, instead, we start at the default state and the system
                % will be controlled towards the random reference.
                
                % Set initial leg, actuator and control states from reference at t=0
                %disp('WARN: Setting initial leg, actuator and control states from random reference at t=0:');
                q_0    = this.ref.random.q_ref(:,1);
                q_d_0  = this.ref.random.q_d_ref(:,1);
                this.setInitialStates(q_0, q_d_0);
                
                % Set flag
                this.ref.use_random = 1;
                
            elseif (use_random == 0)
                % The below often does not work since we require the
                % elongation of the ESB bungees to be zero at t=0 and the
                % corresponding pretension positions are outside the
                % pretension position limits.
                % So, instead, we start at the default state and the system
                % will be controlled towards the reference.
                
                % Set initial leg, actuator and control states from reference at t=0
                %disp('WARN: Setting initial leg, actuator and control states from default reference at t=0.');
                %[q_0, q_d_0] = this.q_ref(0);
                %this.setInitialStates(q_0, q_d_0);
                
                % Set flag
                this.ref.use_random = 0;
                
            else
                error(['Invalid value for use_random: ' num2str(use_random)]);
            end
        end
        
    end
    
end

