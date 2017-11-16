classdef Leg_3DoF_ACA_jumpref_optimizer < handle
    % Leg_3DoF_ACA_optimizer Class for optimization of ESB parameters of the
    % 3-DoF leg actuated by Asymmetric Compliant Actuators (ACAs)
    
    %__________________________________________________________________
    % Properties
    properties
        params      % Optimization parameters
        data        % Optimization data
        sim         % Leg_3DoF_ACA_jumpref_simulation object
        results     % Optimisation results
        list        % Listed iteration data
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_ACA_jumpref_optimizer(actParamsFileName, legParamsFileName)
            % Default parameters
            if (~exist('actParamsFileName', 'var'))
                actParamsFileName = '';
            end
            if (~exist('legParamsFileName', 'var'))
                legParamsFileName = '';
            end
            
            % Set empty struct for results
            this.results = struct;
            
            % Get simulator
            this.sim = Leg_3DoF_ACA_jumpref_simulator(actParamsFileName, legParamsFileName);
           

            
            %NOT NECESSARY, BUT FASTER IF IMPLEMENTED
            % Initial guess TODO LOAD MAT FILE
%             this.data.qinit = %[initial guess in cp] 6x? 
            
            % Control point parameters FIND APPROPRIATE VALUE
            
            this.params.cpres = 300; %Downscale factor (cp = number of q trajectory points / cpres)
                                    
            this.params.t = 0:this.sim.params.Ts:this.sim.params.tspan(2);
            this.params.tcp = this.params.t(1:this.params.cpres:end);
            this.params.n = length((this.params.t(1:this.params.cpres:end)));
            this.params.tn = this.params.n*3;
            
            
            % Listed iteration data
            
            this.list.J_performance =   [];
            this.list.J_stability =     [];
            this.list.J_torque =        [];
            this.list.f =               [];
            this.list.CoM_x =           [];
            this.list.CoM_y =           [];
            
            % Status
            disp('Initialized Leg_3DoF_ACA_jumpref_optimizer with default parameters.');
        end
        
        %__________________________________________________________________
        % Run simulation (tn variables, SQP fmincon)
        function [] = run(this) 
        tic
        %Use q_ref from model for initial simulation
        this.sim.model.ref.use_random = 0; 
            
        % Run initial simulation with initial with q_init  
        disp('Initial simulation')
        this.sim.run(1);    
        
        % Recreate q_init matrix COULD LOAD MAT FILE INSTEAD OF SIM RUN
        t = this.params.t;
        n = this.params.n;
        tn = this.params.tn;
      
        q_init = zeros(length(t),6);q_d_init = zeros(length(t),6);
        for k = 1:length(t)
             [q_init(k,:),q_d_init(k,:)] = this.sim.model.q_ref(t(k)); % q_ref = [q_ref, q_d_ref]
        end
        % Downscale trajectory to control points
        % cp_init = [n x 4] corresponding to [q1 q2 q3 theta1]
        % reshaped to a [4*n,4] vector corresonding to [q1.. q2.. q3.. th..]'
        this.data.qinit = q_init;
        
        fprintf('\n');disp(['Total number of control points: ',num2str(tn)]);
        disp(['Number of control points per joint: ',num2str(n)]);fprintf('\n');
        
        
        cp_init(:,1) = q_init(1:this.params.cpres:end,4); % q1
        cp_init(:,2) = q_init(1:this.params.cpres:end,5); % q2
        cp_init(:,3) = q_init(1:this.params.cpres:end,6); % q3
        
        cp_init = reshape(cp_init,[1,tn]); %Reshape to one long row
        
        % Downscale time to control point time tcp
        tcp = t(1:this.params.cpres:end);
        
        % Run optimization with control points [x] = [cp1 cp2 cp3 cp4] 
        % corresponding to [q1 q2 q3 theta1] 
            
            x0 = cp_init; %initial guess configuration in [4*n,4] vector corresonding to [q1.. q2.. q3.. th..]'

            A=[];
            b=[];
            Aeq=[];
            beq=[];
%             lb= [this.sim.model.leg.params.q_limits(1,1)... % ToDo x tn
%                 this.sim.model.leg.params.q_limits(2,1)...
%                 this.sim.model.leg.params.q_limits(3,1)...
%                 -inf];

            lb = zeros(1,tn); 
            lb(1:n) = this.sim.model.leg.params.q_limits(1,1);          %lb q1
            lb(n+1:2*n) = this.sim.model.leg.params.q_limits(2,1);      %lb q2
            lb(2*n+1:3*n) = this.sim.model.leg.params.q_limits(3,1);    %lb q3

%             ub= [this.sim.model.leg.params.q_limits(1,2)... % ToDo x tn
%                 this.sim.model.leg.params.q_limits(2,2)...
%                 this.sim.model.leg.params.q_limits(3,2)...
%                 inf];

            ub = zeros(1,tn); 
            ub(1:n) = this.sim.model.leg.params.q_limits(1,2);          %ub q1
            ub(n+1:2*n) = this.sim.model.leg.params.q_limits(2,2);      %ub q2
            ub(2*n+1:3*n) = this.sim.model.leg.params.q_limits(3,2);    %ub q3

            % Optimization options
            options = optimoptions('fmincon','Display','iter','Algorithm','sqp'); 

            % Optimization      
%                 X =                             fmincon(      FUN,              X0,A,B,Aeq,Beq,LB,UB,      NONLCON,      OPTIONS)
            [x, fval, exitflag, output, lambda] = fmincon(@(x)this.jumphigh_obj(x),x0,A,b,Aeq,beq,lb,ub,@(x)this.jumpcon(x),options);
            
            this.results.cp = x;
            this.results.f = fval;
            this.results.info = output;
            
            cp(:,1) = this.results.cp(1:n);       %q1
            cp(:,2) = this.results.cp(n+1:2*n);   %q2
            cp(:,3) = this.results.cp(2*n+1:3*n); %q3

            % Construct B-spline trajectory out of control points
            
            % Knots, triple knots at the end
            
            knots = [tcp(1),tcp(1),...
                    tcp(1):tcp(end)/length(tcp):tcp(end),...
                    tcp(end),tcp(end)];
            
            % Least squares fit for the weights of a spline
            s1 = fastBSpline.lsqspline(knots,2,tcp,cp(:,1)); %fastBSpline.lsqspline(knots,order,x,y);
            s2 = fastBSpline.lsqspline(knots,2,tcp,cp(:,2)); %fastBSpline.lsqspline(knots,order,x,y);
            s3 = fastBSpline.lsqspline(knots,2,tcp,cp(:,3)); %fastBSpline.lsqspline(knots,order,x,y);
            
            % Reconstructed q-values
            q1      = s1.evalAt(t);         %[1 x length(t) ]
            q2      = s2.evalAt(t);          
            q3      = s3.evalAt(t);

            this.data.q_res = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3]; 
        toc
        end 
        
        %__________________________________________________________________
        % Constraint function
        function [g, geq] = jumpcon(this,x)     
            g = [];
            geq = [];
        end
       
        %__________________________________________________________________
        % Objective function
        function f = jumphigh_obj(this,x)
            % tn variables optimization 
            % Computation of objective function 

            % Assignment of design variables
            n = this.params.n;
            cp(:,1) = x(1:n);       %q1
            cp(:,2) = x(n+1:2*n);   %q2
            cp(:,3) = x(2*n+1:3*n); %q3

            % Construct B-spline trajectory out of control points
            
            % Knots, triple knots at the end
            
            t = this.params.t;
            tcp = this.params.tcp;
            
            knots = [tcp(1),tcp(1),...
                    tcp(1):tcp(end)/length(tcp):tcp(end),...
                    tcp(end),tcp(end)];
            
            % Least squares fit for the weights of a spline
            s1 = fastBSpline.lsqspline(knots,2,tcp,cp(:,1)); %fastBSpline.lsqspline(knots,order,x,y);
            s2 = fastBSpline.lsqspline(knots,2,tcp,cp(:,2)); %fastBSpline.lsqspline(knots,order,x,y);
            s3 = fastBSpline.lsqspline(knots,2,tcp,cp(:,3)); %fastBSpline.lsqspline(knots,order,x,y);
            
            % Reconstructed q-values
            q1      = s1.evalAt(t);         %[1 x length(t) ]
            q2      = s2.evalAt(t);          
            q3      = s3.evalAt(t);

           this.data.q_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3];
            
            % Check the constructed trajectories for q-limits and
            % work around 
            
            if (    min(q1) < this.sim.model.leg.params.q_limits(1,1)    || ...
                max(q1) > this.sim.model.leg.params.q_limits(1,2)    )
            f = NaN; % skip current iteration 
            fprintf('\n');disp('Joint q1 limit reached: f = NaN');fprintf('\n');
            elseif (    min(q2) < this.sim.model.leg.params.q_limits(2,1)    || ...
                    max(q2) > this.sim.model.leg.params.q_limits(2,2)    )
            f = NaN; % skip current iteration  
           fprintf('\n');disp('Joint q2 limit reached: f = NaN');fprintf('\n');
            elseif (    min(q3) < this.sim.model.leg.params.q_limits(3,1)    || ...
                    max(q3) > this.sim.model.leg.params.q_limits(3,2)    )
            f = NaN; % skip current iteration  
            fprintf('\n');disp('Joint q3 limit reached: f = NaN');fprintf('\n');
            else
            
            % Construct new q_ref matrix (= this.data.qrec)
            this.sim.model.ref.use_random =1;
            q_ref = this.data.q_rec; %[6 x length(t)]
            
            % Calculate reference velocities

            % Cartesian reference
            x_hip       = this.sim.model.ref.x_hip_bias + ...
                            this.sim.model.ref.x_hip_amp * sin(this.sim.model.ref.x_hip_freq * t + this.sim.model.ref.x_hip_phase);
            y_hip       = this.sim.model.ref.y_hip_bias + ...
                            this.sim.model.ref.y_hip_amp * sin(this.sim.model.ref.y_hip_freq * t + this.sim.model.ref.y_hip_phase);

            % Calculate current desired beta, r as a function of time
            beta        = atan2(-x_hip, y_hip);
            r           = sqrt(x_hip.^2 + y_hip.^2);
            
            % Shorthand
            xx = x_hip;
            y = y_hip;

            % Cartesian derivatives
            dxdt        = this.sim.model.ref.x_hip_amp * this.sim.model.ref.x_hip_freq * ...
                            cos(this.sim.model.ref.x_hip_freq * t + this.sim.model.ref.x_hip_phase);
            dydt        = this.sim.model.ref.y_hip_amp * this.sim.model.ref.x_hip_freq * ...
                            cos(this.sim.model.ref.y_hip_freq * t + this.sim.model.ref.y_hip_phase);

            % Partial derivatives
            drdx        = xx ./ sqrt(xx.^2 + y.^2);
            drdy        = y ./ sqrt(xx.^2 + y.^2);
            dbdx        = -1 ./ (y + (xx.^2)./y);
            dbdy        = xx ./ (xx.^2 + y.^2);
            dq4dr       = 1 ./ (this.sim.model.leg.params.l2 * sqrt(4 - (r.^2 ./ this.sim.model.leg.params.l2^2)));
            dq4db       = 1;
            dq5dr       = -2 ./ (this.sim.model.leg.params.l2 * sqrt(4 - (r.^2 ./ this.sim.model.leg.params.l2^2)));
            dq5db       = 0;

            % Intermediate results
            drdt        = drdx .* dxdt + drdy .* dydt;
            dbdt        = dbdx .* dxdt + dbdy .* dydt;

            % Calculate joint velocities
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            this.data.q_d_rec         = zeros(6,length(t));
            this.data.q_d_rec(1,:)    = 0;
            this.data.q_d_rec(2,:)    = 0;
            this.data.q_d_rec(3,:)    = 0;
            this.data.q_d_rec(4,:)    = dq4dr .* drdt + dq4db .* dbdt;
            this.data.q_d_rec(5,:)    = dq5dr .* drdt + dq5db .* dbdt;
            this.data.q_d_rec(6,:)    = -(this.data.q_d_rec(4,:) + this.data.q_d_rec(5,:));

            % Run simulation with new q_ref 
            this.sim.model.ref.random.q_ref = this.data.q_rec;   
            this.sim.model.ref.random.q_d_ref = this.data.q_d_rec;
             
            this.sim.run(0); %%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!
            
           % Check if leg has fallen over  
           if this.sim.outFun(this.sim.data.t,this.sim.data.x_leg) ==1 % This does not work
               % If so, skip current iteration  
               f = NaN;  disp('Leg has fallen over: f = NaN')
           % If not, continue
           elseif this.sim.outFun(this.sim.data.t,this.sim.data.x_leg) ==0    

%             xxx = this.sim.data.x_leg;   
%            N = this.sim.model.leg.N;
%           theta1 = xxx(end-this.sim.model.leg.N+3);                
%            if (theta1 > pi/2 || theta1 < -pi/2)
%                 fprintf('\n');disp('Leg has fallen over: f = NaN');fprintf('\n');
%                 f = NaN;
%            else    
%                
   
            % IK, return tau active and CoM-height    
            [tau_IK,CoM_x,CoM_y] = this.Calc_IK();

             % Performance index CoM height
             J_performance = CoM_y^2;
             fprintf('\n');disp(['J_performance = ',num2str(J_performance)]);
             
             % Stability function CoM 
             J_stability = 1*CoM_x^2; %3e4
             disp(['J_stability = ',num2str(J_stability)]);
             
             % Penalty function tau
             tau_max = 0.085*0.80*30; %k_t1 * r_m1 *i_1_max (ACA_test_params)
             tau_lb = [-tau_max; -tau_max; -tau_max]; %TOO HIGH, TEST VALUES
             tau_ub = [tau_max; tau_max; tau_max];
             tau = tau_IK';    
             Jk = zeros(1,length(tau));  % Preallocate
             for k = 1:length(tau) 
                 Jk(k) = (tau_lb-tau(:,k))'*(tau_lb-tau(:,k))+(tau(:,k)-tau_ub)'*(tau(:,k)-tau_ub);
             end
             J_torque = sum(Jk)*1.8e-9;  % to be scaled appropiately (now scaled with lonely optimum)
             disp(['J_torque = ',num2str(J_torque)]);
             
             % Objective function 
             f = J_stability;
%              f = -J_performance + J_torque + J_stability; 
             fprintf('\n');disp(['Evaluation succeeded: f = ',num2str(f)]);
             disp(['CoM height = ',num2str(CoM_y)]);fprintf('\n'); 
             
            % Add function values to list
            this.list.J_performance = [this.list.J_performance J_performance];
            this.list.J_stability = [this.list.J_stability J_stability];
            this.list.J_torque = [this.list.J_torque J_torque];
            this.list.f = [this.list.f f];
            this.list.CoM_x = [this.list.CoM_x CoM_x];
            this.list.CoM_y = [this.list.CoM_y CoM_y];
            end
            end
             
        end
                
        
        %__________________________________________________________________
        % Inverse Kinematics
        function [tau_IK,CoM_x,CoM_y] = Calc_IK(this)
        
            
        % Get simulation data
        tau         = this.sim.data.tau';

        % Recreate q_ref matrix
        t = 0:this.sim.params.Ts:this.sim.params.tspan(2);
        
        % Insert actual rec data
        q_ref = this.data.q_rec';
        q_d_ref = this.data.q_d_rec';
       

        % IK for q_ref %%% TODO calculate for actual q
        
        %Preallocate
        qp_dd_t_act = zeros(length(t),6);
       
        for k=1:length(t)           %TODO qref to x to qleg makes no sense
            % State and input
            x = [q_ref(k,:)'; q_d_ref(k,:)'];
            u(:,k) = [tau(k,1); tau(k,2); tau(k,3)];

            % No ext forces
            Fe = zeros(3,1);

            % Ground Force F_GRF
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            % Leg: x = [q; q_d]
            q_leg   = x(1 : this.sim.model.leg.N/2); %#ok<*NASGU>
            q_leg_d = x(this.sim.model.leg.N/2 + 1 : this.sim.model.leg.N);

            % Build ground reaction force (GRF)
            % The floor is a spring-damper in the y-direction, and
            % viscous+Coulomb friction in the x-direction.
            % F_GRF = [F_heel_x; F_heel_y; F_toe_x; F_toe_y]
            [~, y_heel, ~]              = this.sim.model.leg.calc_fwdKin_named(q_leg, 'heel');
            [~, y_toe, ~]               = this.sim.model.leg.calc_fwdKin_named(q_leg, 'toe');
            [x_ankle_d, y_ankle_d, ~]   = this.sim.model.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'heel');
            [x_toe_d, y_toe_d, ~]       = this.sim.model.leg.calc_fwdKin_vel_named(q_leg, q_leg_d, 'toe');
             F_ankle_x   = 0;
             F_ankle_y   = 0;
             F_toe_x     = 0;
             F_toe_y     = 0;
             floor_K     = this.sim.model.leg.params.floor_K;
             floor_D     = this.sim.model.leg.params.floor_D;
                if (y_heel <= 0)
                    F_ankle_y   = floor_K * (0.0 - y_heel) + max(0, floor_D * (0.0 - y_ankle_d));
                    coulomb     = -this.sim.model.leg.params.coulomb * tanh(500*x_ankle_d);
                    F_ankle_x   = coulomb + F_ankle_y * this.sim.model.leg.params.mu * (0.0 - x_ankle_d);
                end
                if (y_toe <= 0)
                    F_toe_y     = floor_K * (0.0 - y_toe) + max(0, floor_D * (0.0 - y_toe_d));
                    coulomb     = -this.sim.model.leg.params.coulomb * tanh(500*x_toe_d);
                     F_toe_x     = coulomb + F_toe_y * this.sim.model.leg.params.mu * (0.0 - x_toe_d);
                end

                F_GRF = [F_ankle_x; F_ankle_y; F_toe_x; F_toe_y];


            % State evaluation to obtain active joint accelerations
             x_d = this.sim.model.leg.dx(t(k), x, u(:,k), F_GRF, Fe); %x_d     = [q_d; q_dd];

            % Active joint accelerations
            qa_dd = x_d(10:12);

            % Inverse formulation

            % M-matrices with  M = [Mpp Mpa ; Map Maa]
            M  = this.sim.model.leg.calc_M(q_ref(k,:)); %6x6
            Mpp = M(1:3,1:3);
            Mpa = M(1:3,4:6);
            Map = M(4:6,1:3);
            Maa = M(4:6,4:6);
            % New formulation Ma (to be inverted)
            Ma=[Mpp zeros(3,3); Map -diag(ones(1,3))];
            % New formulation Mb (rhs)
            Mb = [Mpa;Maa];

            

            % Other terms
            G = this.sim.model.leg.calc_G(q_ref(k,:)'); 
            C = this.sim.model.leg.calc_C(q_ref(k,:)',q_d_ref(k,:)'); %
            J_GRF   = this.sim.model.leg.calc_J_GRF(q_ref(k,:)');
            D = this.sim.model.leg.params.d; 
            
            % Inverse formulation
            qp_dd_t_act(k,:) = Ma \ ( -Mb * qa_dd + G - D .* q_d_ref(k,:)' - C * q_d_ref(k,:)' + J_GRF' * F_GRF ); 

        end
        
        tau_IK = qp_dd_t_act(:,4:6);  %Torque active from inverse formulation
        
        %CoM x and y
        for k=1:length(t) 
        [CoM_x(k), CoM_y(k)] = this.sim.model.leg.calc_CoM(q_ref(k,:));
        end
        
        %Last x coordinate CoM
        CoM_x = mean(CoM_x);
        disp('CoM x coordinate');disp(CoM_x);
        %Largest y coordinate CoM
        this.data.CoM_y = CoM_y(:);
        CoM_y = max(CoM_y);
        
        end
        %__________________________________________________________________________
        function plot(this) 
                       
            % Plot original and latest reconstructed trajectories
            figure
            plot(this.params.t, this.data.qinit(:,4)',this.params.t,this.data.q_res(4,:),'--')
            title('q_1');legend('Original','Solution');xlabel('s');ylabel('rad');
            
            figure
            plot(this.params.t, this.data.qinit(:,5)',this.params.t,this.data.q_res(5,:),'--')
            title('q_2');legend('Original','Solution');xlabel('s');ylabel('rad');
            
            figure
            plot(this.params.t, this.data.qinit(:,6)',this.params.t,this.data.q_res(6,:),'--')
            title('q_3');legend('Original','Solution');xlabel('s');ylabel('rad');
            
            % Plot all trajectories
            
            % Plot evolution of f
            figure
            plot(1:length(this.list.f),this.list.f,'o');
            title('Function evolution');xlabel('Objective function evaluation');ylabel('Function value');
        end
        %____________________________
        function animatesolution(this)

            % Calculate velocities
            t = this.params.t;
            
            % Cartesian reference
            x_hip       = this.sim.model.ref.x_hip_bias + ...
                            this.sim.model.ref.x_hip_amp * sin(this.sim.model.ref.x_hip_freq * t + this.sim.model.ref.x_hip_phase);
            y_hip       = this.sim.model.ref.y_hip_bias + ...
                            this.sim.model.ref.y_hip_amp * sin(this.sim.model.ref.y_hip_freq * t + this.sim.model.ref.y_hip_phase);

            % Calculate current desired beta, r as a function of time
            beta        = atan2(-x_hip, y_hip);
            r           = sqrt(x_hip.^2 + y_hip.^2);
            
            % Shorthand
            xx = x_hip;
            y = y_hip;

            % Cartesian derivatives
            dxdt        = this.sim.model.ref.x_hip_amp * this.sim.model.ref.x_hip_freq * ...
                            cos(this.sim.model.ref.x_hip_freq * t + this.sim.model.ref.x_hip_phase);
            dydt        = this.sim.model.ref.y_hip_amp * this.sim.model.ref.x_hip_freq * ...
                            cos(this.sim.model.ref.y_hip_freq * t + this.sim.model.ref.y_hip_phase);

            % Partial derivatives
            drdx        = xx ./ sqrt(xx.^2 + y.^2);
            drdy        = y ./ sqrt(xx.^2 + y.^2);
            dbdx        = -1 ./ (y + (xx.^2)./y);
            dbdy        = xx ./ (xx.^2 + y.^2);
            dq4dr       = 1 ./ (this.sim.model.leg.params.l2 * sqrt(4 - (r.^2 ./ this.sim.model.leg.params.l2^2)));
            dq4db       = 1;
            dq5dr       = -2 ./ (this.sim.model.leg.params.l2 * sqrt(4 - (r.^2 ./ this.sim.model.leg.params.l2^2)));
            dq5db       = 0;

            % Intermediate results
            drdt        = drdx .* dxdt + drdy .* dydt;
            dbdt        = dbdx .* dxdt + dbdy .* dydt;

            % Calculate joint velocities
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            this.data.q_d_res         = zeros(6,length(t));
            this.data.q_d_res(1,:)    = 0;
            this.data.q_d_res(2,:)    = 0;
            this.data.q_d_res(3,:)    = 0;
            this.data.q_d_res(4,:)    = dq4dr .* drdt + dq4db .* dbdt;
            this.data.q_d_res(5,:)    = dq5dr .* drdt + dq5db .* dbdt;
            this.data.q_d_res(6,:)    = -(this.data.q_d_res(4,:) + this.data.q_d_res(5,:));
            
            % Run simulation with new q_ref 
            this.sim.model.ref.random.q_ref = this.data.q_res;   
            this.sim.model.ref.random.q_d_ref = this.data.q_d_res;
            fprintf('\n');disp('Resimulating for optimal solution');
            this.sim.run(1);
            fprintf('\n');disp('Rerun animation with this.sim.animate');
        end
    end %end methods
end %end classdef























