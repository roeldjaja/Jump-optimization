classdef ACA < handle
%ACA Asymmetric Compliant Actuator (ACA) model class
% We implement the dynamics with current inputs, to remove the need to
% implement a current controller. This removes the electrical dynamics from
% the system, but they should be negligible. The required motor voltage to
% achieve this current is calculated by taking the back-EMF into account
% and using the electrical resistance of the windings, thus allowing to
% easily calculate the electrical power.
    
    %__________________________________________________________________
    % Private properties
    properties
    end
    
    
    %__________________________________________________________________
    % Public properties
    properties
        params              % Parameters struct
        N                   % Number of states N
        M                   % Number of inputs M
        K                   % Number of outputs K
        x0                  % Initial conditions
        inputIdx_q_d        % Indexes in input u that correspond to joint velocities
        outputIdx_P_1       % Index in output y that corresponds to P_1
        outputIdx_P_2       % Index in output y that corresponds to P_2
        outputIdx_v_1       % Index in output y that corresponds to v_1
        outputIdx_v_2    	% Index in output y that corresponds to v_2
        outputIdx_tau_pb    % Indexes in output y that correspond to tau_pb
        outputIdx_tau_p     % Indexes in output y that correspond to tau_p
        outputIdx_tau       % Indexes in output y that correspond to tau
        % x = [x_1_d; p_d; dL_pb, dL_p; x_1; p]
        stateIdx_x_1_d      % x_1_d state index
        stateIdx_p_d        % p_d state index
        stateIdx_dL_pb      % dL_pb state index
        stateIdx_dL_p       % dL_p state index
        stateIdx_x_1        % x_1 state index
        stateIdx_p          % p state index
        topology            % Topology struct
        type                % ACA type (mono, bi, tri, noESB)
        control             % Control parameters/variables
    end
    
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = ACA(paramsName)
            % Load parameters
            this.params = eval(paramsName);
            
            % Set state-space dimensions
            this.N = 6;             % Number of states N
            this.M = 3;             % Number of inputs M (default, depends on dimension of q_d)
            this.K = this.N+7;      % Number of outputs K (default, depends on dimension of q_d) 
            this.x0 = zeros(this.N,1);          % Initial conditions
            this.inputIdx_q_d       = 3;      	% Indexes in input u that correspond to joint velocities (default, depends on dimension of q_d) 
            this.outputIdx_P_1      = this.N+1;	% Index in output y that corresponds to P_1
            this.outputIdx_P_2      = this.N+2;	% Index in output y that corresponds to P_2
            this.outputIdx_v_1      = this.N+3; % Index in output y that corresponds to v_1
            this.outputIdx_v_2      = this.N+4; % Index in output y that corresponds to v_2
            % The outputIdx fields below here (and this.K) are updated by setTopology.
            this.outputIdx_tau_pb   = this.N+5;	% Indexes in output y that correspond to tau_pb (default, depends on dimension of q_d) 
            this.outputIdx_tau_p    = this.N+6;	% Indexes in output y that correspond to tau_p (default, depends on dimension of q_d) 
            this.outputIdx_tau      = this.N+7; % Indexes in output y that correspond to tau (default, depends on dimension of q_d)
            this.stateIdx_x_1_d     = 1;        % x_1_d state index
            this.stateIdx_p_d       = 2;        % p_d state index
            this.stateIdx_dL_pb     = 3;        % dL_pb state index
            this.stateIdx_dL_p      = 4;        % dL_p state index
            this.stateIdx_x_1       = 5;        % x_1 state index
            this.stateIdx_p         = 6;        % p state index
            
            % Set default topology (1-DoF system, monoarticulated ESB)
            this.setTopology(-1,1);
            
            % For low-pass filters / exp. smoothing below
            Ts      = 1e-3;
            f_c     = 200;      % [Hz]
            x       = (2 * pi * Ts * f_c);
            a_200   = x / (x+1);
            f_c     = 0.015;    % [Hz]
            x       = (2 * pi * Ts * f_c);
            a_0015  = x / (x+1);
            
            % Set default control parameters
            this.control.Ts             = Ts; 	    % Controller timestep [s] (1 KHz)
            this.control.k_m            = 250;      % PB proportional gain
            this.control.d_m            = 0.02;     % PB derivative gain
            this.control.a_m            = a_200;    % PB derivative exp. smooth. factor
            this.control.k_pp           = 3.0;      % ESB proportional gain
            this.control.d_pp           = 0.15;     % ESB derivative gain
            this.control.a_pp           = a_200;    % ESB derivative exp. smooth. factor
            this.control.a_p_ref        = a_0015;  	% Pretension position ref. exp. smooth. factor
            
            % Set default control variables
            this.control.dL_pb_ref_prev	= 0;        % PB elongation ref.
            this.control.p_opt_prev     = 0;        % Pretension opt. pos. ref.
            this.control.p_ref_prev     = 0;     	% Pretension pos. ref.
            this.control.e_m_d_prev     = 0;        % PB error derivative
            this.control.e_p_d_prev     = 0;        % ESB error derivative
            this.control.i_1            = 0;        % PB current
            this.control.i_2            = 0;        % ESB current
        end
        
        
        %__________________________________________________________________
        % Set incidence / actuator topology matrix and its index
        function setTopology(this, T, idx)
            % Set actuator topology matrix and index
            this.topology.T         = T;
            this.topology.idx       = idx;
            
            % Get number of joints N and number of actuators M
            % T is of dimension MxN
            this.topology.N         = size(T,2);
            this.topology.M         = size(T,1);
            
            % Obtain this actuator's topology vector and its PB topology
            % vector. The latter is merely a selector to select the correct
            % joint for the PB elongation and sets its output torque
            % appropriately.
            
            % Row vector t, transforming N joint velocities into this
            % actuator's ESB elongation
            this.topology.t         = T(idx,:);
            
            % Set actuator type (mono, bi, tri, noESB)
            art = length(nonzeros(this.topology.t)); % Find nonzero elements in row
            % We only support no ESB, mono-, bi- and tri-articulation
            % for now
            switch (art)
                case 0
                    % No ESB
                    this.type = 'noESB';
                case 1
                    % Monoarticulation
                    this.type = 'mono';
                case 2
                    % Biarticulation
                    this.type = 'bi';
                case 3
                    % Triarticulation
                    this.type = 'tri';
                otherwise
                    error(['Unsupported articulation: ' num2str(T)]);
            end
            
            % Row vector t_PB, transforming N joint velocities into this
            % actuator's PB elongation (all zero, except for the driven
            % joint). We set -1, as positive joint velocity compresses the
            % element.
            t_pb                    = zeros(1,this.topology.N);
            t_pb(idx)               = -1;
            this.topology.t_pb      = t_pb;
            
            % Set the number of inputs of the system corresponding to the
            % number of joints
            this.M                  = 2 + this.topology.N;	% Two currents + all joint velocities
            this.inputIdx_q_d       = 3:this.M;             % All inputs are [q_d]
            
            % Set the number of outputs of the system corresponding to the
            % number of joints
            this.K                  = this.N + 4 + 3*this.topology.N; % 3x, for [tau_pb, tau_p, tau]
            % Update outputIdx values based on topology
            this.outputIdx_tau_pb   = this.outputIdx_tau_pb : this.outputIdx_tau_pb-1+this.topology.N;
            this.outputIdx_tau_p    = max(this.outputIdx_tau_pb)+1 : max(this.outputIdx_tau_pb)+this.topology.N;
            this.outputIdx_tau      = max(this.outputIdx_tau_p)+1 : max(this.outputIdx_tau_p)+this.topology.N;
        end
        
        
        %__________________________________________________________________
        % Evolve dynamics given time t, state x and input u
        function dx = dx(this, t, x, u) %#ok<*INUSL>
            % Get linear and nonlinear parts
            [A, B, ~, ~]	= this.getStateSpaceMatrices();
            [f, ~]          = this.getNonlinearDynamics(t, x, u);
            
            % dx = A * x + B * u + f(x,u)
            dx = A * x + B * u + f;
        end
        
        
        %__________________________________________________________________
        % Get the output y
        function y = y(this, t, x, u) %#ok<*INUSL>
            % Get linear and nonlinear parts
            [~, ~, C, D]	= this.getStateSpaceMatrices();
            [~, g]          = this.getNonlinearDynamics(t, x, u);
            
            % y = C * x + D * u + g(x,u)
            for i=1:length(t)
                y(:,i) = C * x(:,i) + D * u(:,i) + g(:,i);
            end
        end
        
        
        %__________________________________________________________________
        % Get nonlinear dynamics f(x,u) and g(x,u)
        function [f, g] = getNonlinearDynamics(this, t, x, u) %#ok<*INUSD>
            % x = [x_1_d; p_d; dL_pb, dL_p; x_1; p]
            % u = [i_1, i_2, [q_d]] where q_d is a topology.Nx1 vector
            % y = [[x]; P_1; P_2; v_1; v_2; [tau_pb]; [tau_p]; [tau]]
           
            % Get ESB elongation
            dL_p = x(4);
          
            % Check pretension range out of bounds, after checking for
            % noESB
            % Check for noESB
            r = -this.topology.t(this.topology.idx);  % r
            if (abs(r) > 0)
                p           = x(6);
                p_range     = this.params.p_range;
                if (p < p_range(1) || p > p_range(2))
                    p_range %#ok<NOPRT>
                    error(['ACA ERR: Pretension position p = ' num2str(p) ' out of range p_range = [' num2str(p_range) '] for ACA with index ' num2str(this.topology.idx) '!']);
                end
            end
            
            % Create unidirectional spring force (by compensating for the
            % corresponding term in the linear dynamics)
            f = zeros(this.N, 1); % Dynamics
            g = zeros(this.K, 1); % Outputs
            
            % If there is negative extension (i.e. slack),
            % disable/compensate for the linear spring force:
            if (dL_p <= 0)
                % Parameter shorthands
                r_m2	= this.params.r_m2;
                k_p     = this.params.k_p;
                I_m2    = this.params.I_m2;
                
                % Dynamics
                % == -A(2,4)
                % Disable this for bidirectional spring behaviour
                % NOTE: Now disabled as we force p to stay constant (I.e. we require p_d == 0)
                %f(2) = k_p * dL_p / (I_m2 * r_m2^2);
                
                % Outputs
                % == -C(this.outputIdx_tau_p, 4) and -C(this.outputIdx_tau, 4)
                % Disable this for bidirectional spring behaviour
                g(this.outputIdx_tau_p) = this.topology.t' * k_p * dL_p;	% tau_p (from dL_p)
                g(this.outputIdx_tau) 	= g(this.outputIdx_tau_p);          % tau (from dL_p)
            end
            
            % Get states, inputs
            i_1 = u(1,:);       i_2 = u(2,:);
            x_1_d = x(1,:);     p_d = x(2,:);

            % Parameter shorthands
            d_e1 = this.params.d_e1;
            k_t1 = this.params.k_t1;
            r_m1 = this.params.r_m1;
            d_e2 = this.params.d_e2;
            k_t2 = this.params.k_t2;
            r_m2 = this.params.r_m2;
            
            % Calculate motor power outputs
            v_1_max     = this.params.v_1_max; % PB max voltage [V]
            v_2_max     = this.params.v_2_max; % ESB max voltage [V]
            for i=1:length(t)
                % Calculate nonlinear electrical motor power output
                % v = i*R + back-EMF voltage
                % v_1 = (i_1 * d_e1 + k_t1 * r_m1 * x_1_d);
                % v_2 = (i_2 * d_e2 + k_t2 * r_m2 * p_d);
                v_1 = i_1(i) * d_e1 + k_t1 * r_m1 * x_1_d(i);
                v_2 = i_2(i) * d_e2 + k_t2 * r_m2 * p_d(i);
                
                % Limit voltage to max applicable voltage
                if (abs(v_1) > v_1_max) % PB
                    v_1 = sign(v_1) * v_1_max;
                    i_1 = (v_1 - k_t1 * r_m1 * x_1_d) / d_e1;
                end
                if (abs(v_2) > v_2_max) % ESB
                    v_2 = sign(v_2) * v_2_max;
                    i_2 = (v_2 - k_t2 * r_m2 * p_d) / d_e2;
                end
                
                % P_1 = i_1 * v_1;
                % P_2 = i_2 * v_2;
                g(this.outputIdx_P_1,i) = i_1(i) * v_1;
                g(this.outputIdx_P_2,i) = i_2(i) * v_2;
                g(this.outputIdx_v_1,i) = v_1;
                g(this.outputIdx_v_2,i) = v_2;
            end
        end
        
        
        %__________________________________________________________________
        % Get state-space matrices (linear part of model)
        function [A, B, C, D] = getStateSpaceMatrices(this)
            % Parameter shorthands
            % PB motor
            %d_e1 = this.params.d_e1;
            %H_1  = this.params.H_1;
            k_t1 = this.params.k_t1;
            I_m1 = this.params.I_m1;
            d_m1 = this.params.d_m1;

            % ESB motor
            %d_e2 = this.params.d_e2;
            %H_2  = this.params.H_2;
            k_t2 = this.params.k_t2;
            I_m2 = this.params.I_m2;
            d_m2 = this.params.d_m2;

            % Ballscrew and Harmonic Drive transmissions (including motor gearbox)
            r_m1 = this.params.r_m1;
            r_m2 = this.params.r_m2;

            % PB spring
            k_pb = this.params.k_pb;
            d_pb = this.params.d_pb;

            % ESB spring
            k_p = this.params.k_p;
            d_p = this.params.d_p;
            
            
            % Initialise matrices
            A = zeros(this.N);          % NxN
            B = zeros(this.N, this.M);  % NxM
            C = zeros(this.K, this.N);  % KxN
            D = zeros(this.K, this.M);  % KxM
            
            
            % A
            % x = [x_1_d; p_d; dL_pb, dL_p; x_1; p]
            
            % x_1_d
            A(1,1) = -(d_m1 * r_m1^2 + d_pb) / (I_m1 * r_m1^2);
            A(1,3) = -k_pb / (I_m1 * r_m1^2);
            
            % p_d
%             A(2,2) = -(d_m2 * r_m2^2 + d_p) / (I_m2 * r_m2^2);
%             A(2,4) = -k_p / (I_m2 * r_m2^2);

            
            % dL_pb
            A(3,1) = 1;
            
            % dL_p
            A(4,2) = 1;
            
            % x_1
            A(5,1) = 1;
            
            % p
            A(6,2) = 1;
            
            
            % B
            % u = [i_1, i_2, [q_d]] where q_d is a topology.Nx1 vector
            
            % x_1_d
            B(1,1) = k_t1 * r_m1 / (I_m1 * r_m1^2);
            B(1,this.inputIdx_q_d) = -this.topology.t_pb * d_pb / (I_m1 * r_m1^2);
            
            % p_d
%             B(2,2) = k_t2 * r_m2 / (I_m2 * r_m2^2);
%             B(2,this.inputIdx_q_d) = -this.topology.t * d_p / (I_m2 * r_m2^2);

            
            % dL_pb
            B(3,this.inputIdx_q_d) = this.topology.t_pb;
            
            % dL_p
            B(4,this.inputIdx_q_d) = this.topology.t;
         
            
            % C
            % x = [x_1_d; p_d; dL_pb, dL_p; x_1; p]
            % y = [[x]; P_1; P_2; v_1; v_2; [tau_pb]; [tau_p]; [tau]]
            
            C(1:this.N, 1:this.N) = eye(this.N);
            C(this.outputIdx_tau_pb, 1) = -this.topology.t_pb' * d_pb;  % tau_pb (from x_1_d)
            C(this.outputIdx_tau_pb, 3)	= -this.topology.t_pb' * k_pb;	% tau_pb (from dL_pb)
            C(this.outputIdx_tau_p, 2)  = -this.topology.t' * d_p;      % tau_p (from p_d)
            C(this.outputIdx_tau_p, 4)	= -this.topology.t' * k_p;      % tau_p (from dL_p)
            C(this.outputIdx_tau, 1)    = C(this.outputIdx_tau_pb, 1);	% tau (from dL_pb)
            C(this.outputIdx_tau, 2)    = C(this.outputIdx_tau_p, 2);  	% tau (from dL_p)
            C(this.outputIdx_tau, 3)    = C(this.outputIdx_tau_pb, 3);	% tau (from dL_pb)
            C(this.outputIdx_tau, 4)    = C(this.outputIdx_tau_p, 4);  	% tau (from dL_p)
            
            
            % D
            % u = [i_1, i_2, [q_d]] where q_d is a topology.Nx1 vector
            % y = [[x]; P_1; P_2; v_1; v_2; [tau_pb]; [tau_p]; [tau]]
            
            % Direct feed-through for joint motion to elastic element
            % damping torque
            D(this.outputIdx_tau_pb, this.inputIdx_q_d)	= -this.topology.t_pb' * this.topology.t_pb * d_pb;	% tau_pb (from dL_pb)
            D(this.outputIdx_tau_p, this.inputIdx_q_d)	= -this.topology.t' * this.topology.t * d_p;        % tau_p (from dL_p)
            D(this.outputIdx_tau, this.inputIdx_q_d)    = D(this.outputIdx_tau_pb, this.inputIdx_q_d);      % tau (from dL_pb)
            D(this.outputIdx_tau, this.inputIdx_q_d)    = D(this.outputIdx_tau_p, this.inputIdx_q_d);       % tau (from dL_p)
            
        end
        
        
        %__________________________________________________________________
        % Get state-space system (linear part of model)
        function sys = getStateSpace(this)
            % Get state-space matrices and build ss
            [A, B, C, D] = this.getStateSpaceMatrices();
            sys = ss(A, B, C, D);
            
            % Set state, input and output names
            
            % States
            % x = [x_1_d; p_d; dL_pb, dL_p; x_1; p]
            sys.StateName = {'x_{1,d}', 'p_d', 'dL_{pb}', 'dL_p', 'x_1', 'p'};
            
            % Inputs
            % u = [i_1, i_2, [q_d]] where q_d is a topology.Nx1 vector
            for i=1:this.topology.N
                q_d_names{i} = ['q_d[' num2str(i) ']']; %#ok<*AGROW>
            end
            a = horzcat({'i_1', 'i_2'}, q_d_names);
            sys.InputName = a;
            
            % Outputs
            % y = [[x]; P_1; P_2; v_1; v_2; [tau_pb]; [tau_p]; [tau]]
            for i=1:this.topology.N
                torqueOutputNames{i}	= ['tau_{pb}[' num2str(i) ']'];
                torqueOutputNames{this.topology.N+i}	= ['tau_p[' num2str(i) ']'];
                torqueOutputNames{2*this.topology.N+i}  = ['tau[' num2str(i) ']'];
            end
            sys.OutputName = horzcat(   sys.StateName', ...
                                        {'P_1', 'P_2', 'v_1', 'v_2'}, ...
                                        torqueOutputNames                   );
        end
        
        
        %__________________________________________________________________
        % Get control action
        function [i_1, i_2] = controller(this, t, x, tau_ref, q, q_d, tau_p, tau_s)
            
%              x(2) %p_d
%              x(6) %p
            
            % Get the ESB torques on this ACA's driven joint index
            tau_p = tau_p(this.topology.idx); % Net ESB torque
            tau_s = tau_s(this.topology.idx); % ESB torque from other (spanned) actuators
            
            % Torque distribution
            % tau_pb_ref = tau_ref - tau_p (where tau_p = full ESB torque)
            % tau_p_ref = tau_ref - tau_s (where tau_s = spanned ESB torques)
            % from other actuators)
            tau_pb_ref	= tau_ref - tau_p;
            tau_p_ref   = tau_ref - tau_s;
            
            % Controller timestep (for calculation of derivatives)
            Ts          = this.control.Ts;
            
            % PB control law
            % dL_pb_ref = K^-1 * tau_pb_ref (deflection control)
            
            % e_m = dL_pb_ref - dL_pb
            % e_m_d = -dL_pb_d = -(x_1_d - q_d)
            % tau_m1 = r_m1^-1 * k_pb * dL_pb + k_m * (e_m + d_m * e_m_d)
            x_1_d       = x(1);
            k_m         = this.control.k_m;             % PB proportional gain
            d_m         = this.control.d_m;             % PB derivative gain
            k_pb        = this.params.k_pb;             % k_pb
            r_m1        = this.params.r_m1;             % r_m1
            dL_pb_ref	= k_pb^-1 * tau_pb_ref;         % Des. elongation from des. torque
            dL_pb_ref_prev = this.control.dL_pb_ref_prev;                   % Previous deflection ref.
            dL_pb_ref_d = (dL_pb_ref - dL_pb_ref_prev) * Ts;                % Deflection ref. derivative
            dL_pb       = x(3);                                             % dL_pb
            e_m         = dL_pb_ref - dL_pb;                                % Deflection error
            e_m_d       = dL_pb_ref_d - (x(1) + this.topology.t_pb * q_d);	% Deflection error derivative
            e_m_d_prev  = this.control.e_m_d_prev;                          % Prev. error derivative

            a_m         = this.control.a_m;                               	% Exp. smooth. factor
            e_m_d       = a_m * e_m_d + (1 - a_m) * e_m_d_prev;           	% Filtered error derivative
            tau_m1      =   r_m1^-1 * tau_pb_ref + ...      % Feed-forward
                            k_m * (e_m + d_m * e_m_d);      % PD
                            %r_m1^-1 * k_pb * dL_pb + ...	% Compensation
            
            % ESB control law
            % p_opt = -t_i * q + (r * k_p)^-1 * tau_p_ref (where t_i
            % denotes this actuator's actuation topology vector, and q
            % denotes the current N-DoF joint positions)
            % e_p       = p_ref - p
            % e_p_d     = p_d_ref - p_d
            % tau_m2	= r_m2^-1 * k_p * dL_p + k_pp * (e_p + d_pp * e_p_d)
            k_pp        = this.control.k_pp;                    % ESB proportional gain
            d_pp        = this.control.d_pp;                    % ESB derivative gain
            k_p         = this.params.k_p;                      % k_p
            r_m2        = this.params.r_m2;                     % r_m2
            r           = -this.topology.t(this.topology.idx);  % r
            dL_p_ref    = (r * k_p)^-1 * tau_p_ref;             % Elongation reference
            
            % Check for noESB
            if (abs(r) > 0)
                p_opt       = -this.topology.t * q + dL_p_ref;
            else
                p_opt       = 0; % noESB
            end
            
            % Filter optimal pretension position reference
            p_opt_prev	= this.control.p_opt_prev;                      % Previous p_opt
            a_p_ref     = this.control.a_p_ref;                         % Exp. smooth. factor
            p_opt       = a_p_ref * p_opt + (1-a_p_ref) * p_opt_prev;	% Low-pass filtering for p_opt
            
            % Slack control
            % Calculate zero pretension position, and set p_ref accordingly
            p_zp        = -this.topology.t * q;
            p_ref       = max(p_opt, p_zp);
            
            %p_ref       = this.x0(6)+0.05; % Fixed pretension position
            %p_ref       = this.params.p_static; % Fixed at optimized pretension pos.
            
            % Check pretension reference against possible range and
            % saturate, after checking for noESB
            % Check for noESB
            if (abs(r) > 0)
                p_range     = this.params.p_range;
                offset      = 0.01; % Offset 1cm from limits
                if (p_ref < p_range(1)+offset)
                    p_ref = p_range(1)+offset;
                elseif (p_ref > p_range(2)-offset)
                    p_ref = p_range(2)-offset;
                end
            end
            
            % Calculate ESB control action
            p           = x(6);                                 % Pretension pos.
            p_d         = x(2);                                 % Pretension pos. derivative
            p_ref_prev  = this.control.p_ref_prev;              % Prev. pretension pos. ref.
            p_d_ref     = (p_ref - p_ref_prev) * Ts;            % p_ref derivative
%             e_p         = p_ref - p;                            % Error
            e_p         = 0;                            % Error = 0
%             e_p_d       = p_d_ref - p_d;                        % Error derivative
            e_p_d       = 0;                      % Error derivative = 0
            e_p_d_prev  = this.control.e_p_d_prev;              % Prev. error set to zero

            a_pp        = this.control.a_pp;                    % Exp. smooth. factor
            e_p_d       = a_pp * e_p_d + (1-a_pp) * e_p_d_prev;	% Filtered error derivative
            dL_p        = x(4);                                 % ESB elongation
            if (dL_p > 0)
%                 tau_m2      =   r_m2^-1 * k_p * dL_p + ...      % Compensation
%                                 k_pp * (e_p + d_pp * e_p_d);    % PD
                tau_m2      =   0 + ...                         % Compensation
                                k_pp * (e_p + d_pp * e_p_d);    % PD

            else
                tau_m2      =	k_pp * (e_p + d_pp * e_p_d);    % PD
            end
            
            % Currents from torques
            d_e1    = this.params.d_e1;
            k_t1    = this.params.k_t1;
            r_m1    = this.params.r_m1;
            d_e2    = this.params.d_e2;
            k_t2    = this.params.k_t2;
            r_m2    = this.params.r_m2;
            i_1     = tau_m1 / k_t1;
            i_2     = tau_m2 / k_t2;
            
            % Current limitation
            if (abs(i_1) > this.params.i_1_max)
                i_1 = sign(i_1) * this.params.i_1_max;
            end
            if (abs(i_2) > this.params.i_2_max)
                i_2 = sign(i_2) * this.params.i_2_max;
            end
            
            % Limit current based on max applicable voltage
            % v = i*R + back-EMF voltage
            % v_1 = (i_1 * d_e1 + k_t1 * r_m1 * x_1_d);
            % v_2 = (i_2 * d_e2 + k_t2 * r_m2 * p_d);
            v_1         = i_1 * d_e1 + k_t1 * r_m1 * x_1_d;
            v_2         = i_2 * d_e2 + k_t2 * r_m2 * p_d;
            v_1_max     = this.params.v_1_max; % PB max voltage [V]
            v_2_max     = this.params.v_2_max; % ESB max voltage [V]
            if (abs(v_1) > v_1_max) % PB
                i_1 = (sign(v_1) * v_1_max - k_t1 * r_m1 * x_1_d) / d_e1;
            end
            if (abs(v_2) > v_2_max) % ESB
                i_2 = (sign(v_2) * v_2_max - k_t2 * r_m2 * p_d) / d_e2;
            end
            
            % Save values for next iteration
            this.control.dL_pb_ref_prev = dL_pb_ref;        % PB elongation ref.
            this.control.p_opt_prev     = p_opt;            % Pretension opt. pos. ref.
            this.control.p_ref_prev     = p_ref;            % Pretension pos. ref.
            this.control.e_m_d_prev     = e_m_d;            % PB error derivative
            this.control.e_p_d_prev     = e_p_d;            % ESB error derivative
            this.control.i_1            = i_1;              % PB current
            this.control.i_2            = i_2;              % ESB current
        end
        
    end
    
end