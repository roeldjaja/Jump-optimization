classdef Leg_3DoF_ACA_jumpref_optimizer < handle
    % Leg_3DoF_ACA_jumpref_optimizer Class for optimization of the trajectory of the
    % 3-DoF leg actuated by Asymmetric Compliant Actuators (ACAs)
    
    % Construction example for biarticulated:
    % <...> = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_biarticulated.mat','Leg_3DoF_design_bi')
    % Run optimization with this.run, 
    % Afterwards resimulate solution with this.simulate_solution and use this.sim.animate to animate latest simulation
    % Plot optimization data with this.plot
    
    % Available Data     
    % All evaluation references         this.list.q_rec            
    % High jump criterion values        this.list.J_high   
    % Efficient jump criterion values   this.list.J_energy   
    % Stability criterion values        this.list.J_stability      
    % Torque criterion values           this.list.J_torque        
    % Objective function values         this.list.f     
    % cumulative electrical energy      this.list.E
    % CoM y coordinates (max)           this.list.CoM_y                        
    % All evaluation ground forces      this.list.F_GRF  
    % Control points                    this.list.cp 
    % Pretensions                       this.list.p
    % Leg state q                       this.list.q_leg 
    % Leg state q_d                     this.list.q_leg_d 
    
    % TODO: sufficient change in p variables
    
    %__________________________________________________________________
    % Properties
    properties
        params      % Optimization parameters
        data        % Optimization data
        sim         % Leg_3DoF_ACA_jumpref_simulation object
        results     % Optimisation results
        list        % Listed iteration data
        plots       % Plotting parameters & colours
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_ACA_jumpref_optimizer(actParamsFileName, legParamsFileName)
            % Default parameters
            this.params.noESB = 0;
            if (~exist('actParamsFileName', 'var'))
                actParamsFileName = '';
                % Recognize ESB
                this.params.noESB = 1;
            end
            if (~exist('legParamsFileName', 'var'))
                legParamsFileName = '';
            end
            if (exist('legParamsFileName', 'var'))
                % Recognize articulation
                this.params.leginput = legParamsFileName;
            end
            
            % Set empty struct for results
            this.results = struct;
            
            % Get simulator
            this.sim = Leg_3DoF_ACA_jumpref_simulator(actParamsFileName, legParamsFileName);
            
            % Objective criteria weights
            
            % Performance (high)
            this.params.c_high  = 5e2;
            
            % Energy
            this.params.c_ener  = 2e-6;

            % Reference height
            this.params.c_y_ref = 1e1;
            
            % Stability
            this.params.c_xh    = 1.5e3;      %1e4  % CoM_x corresponding to highest CoM_y
            this.params.c_xm    = 1e2;        % Mean CoM_x
            this.params.c_RM    = 1;        % Sum Rotational momentum around CoM
            
            % Torque
            this.params.c_torq  = 2e-4;
            
            % Time
            this.params.t = 0 : this.sim.params.Ts : this.sim.params.tspan(2);
            
            % Control point parameters 
            this.params.cpres           = 100;       % Get cp at t(1) and every t(cpres+1)  
            
            % Minimal Jumping height
            this.params.CoM_y_ref       = 0.75;
            
            % Initial pretension positions (third = 0, is reset in optimization)
            this.data.p_init = [0.0001 0.0001 eps];
            
            % Optimization options
            this.params.DiffMinChange           = 1e-3;     % Default 0
            this.params.DiffMaxChange           = 0.5;      % Default inf
            this.params.InitTrustRegionRadius   = 1;        % Default sqrt(number of variables)
            this.params.MaxFunctionEvaluations  = 6000;     % Default 3000
            
            % Status
            disp('Initialized Leg_3DoF_ACA_jumpref_optimizer with default parameters.');
            
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
        end
        
        %__________________________________________________________________
        % Run optimization (tn variables, SQP fmincon)
        function [] = run(this)
            
            trun = tic;
            
            % Listed iteration data
            
            this.list.q_rec         = [];
            this.list.J_high        = [];
            this.list.J_energy      = [];
            this.list.J_stability   = [];
            this.list.J_torque      = [];
            this.list.f             = [];
            this.list.E             = [];
            this.list.CoM_y         = [];
            this.list.q_rec         = [];
            this.list.F_GRF         = [];
            this.list.cp            = [];
            this.list.p             = [];
            
            % Control point parameters
            this.params.tcp = this.params.t(1 : this.params.cpres : end);
            this.params.n   = length((this.params.t(1 : this.params.cpres : end)));
            this.params.tn  = this.params.n*3;
            
            % Time and numbers shorthand
            t   = this.params.t;
            n   = this.params.n;
            tn  = this.params.tn;
            Ts  = this.sim.params.Ts;
            
            % Initial guess possible todo load mat file
            load('q_init','q_init')
            
            this.data.q_init = q_init;
            
            % Downscale trajectory to control points
            % cp_init = [n x 3] corresponding to [q1 q2 q3]
            % reshaped to a [1,3*n] vector corresonding to [q1.. q2.. q3..]'
            
            fprintf('\n');disp(['Total number of control points: ',num2str(tn)]);
            disp(['Number of control points per joint: ',num2str(n)]);fprintf('\n');
            
            
            cp_init(:,1) = q_init(1:this.params.cpres:end,4); % q1
            cp_init(:,2) = q_init(1:this.params.cpres:end,5); % q2
            cp_init(:,3) = q_init(1:this.params.cpres:end,6); % q3
            cp_init = reshape(cp_init,[1,tn]); %Reshape to one long row
            
            % Downscale time to control point time tcp
            tcp = t(1:this.params.cpres:end);
            
            %Optimize without ESB
            if this.params.noESB == 1
                % Run optimization with control points [x] = [cp1 cp2 cp3]
                % corresponding to [q1 q2 q3]

                x0  = cp_init; %initial guess configuration in [3*n,1] vector corresonding to [q1.. q2.. q3..]'

                A   = [];
                b   = [];
                Aeq = [];
                beq = [];

                lb              = zeros(1,tn);
                lb(1:n)         = this.sim.model.leg.params.q_limits(1,1);    %lb q1
                lb(n+1:2*n)     = this.sim.model.leg.params.q_limits(2,1);    %lb q2
                lb(2*n+1:3*n)   = this.sim.model.leg.params.q_limits(3,1);    %lb q3

                ub              = zeros(1,tn);
                ub(1:n)         = this.sim.model.leg.params.q_limits(1,2);    %ub q1
                ub(n+1:2*n)     = this.sim.model.leg.params.q_limits(2,2);    %ub q2
                ub(2*n+1:3*n)   = this.sim.model.leg.params.q_limits(3,2);    %ub q3

                % Optimization options
                options = optimoptions( 'fmincon',...           
                                        'Display','iter',...
                                        'Algorithm','interior-point',...
                                        'InitTrustRegionRadius',this.params.InitTrustRegionRadius,...
                                        'DiffMinChange',this.params.DiffMinChange,...
                                        'DiffMaxChange',this.params.DiffMaxChange,...
                                        'MaxFunctionEvaluations', this.params.MaxFunctionEvaluations,...
                                        'TypicalX',cp_init);%,...
                                        % 'FiniteDifferenceStepSize',1e-3,...

                % Optimization
                [x, fval, exitflag, output, lambda] = fmincon(@(x)this.jumphigh_obj(x),x0,A,b,Aeq,beq,lb,ub,@(x)this.jumpcon(x),options);

                this.results.cp     = x;
                this.results.f      = fval;
                this.results.info   = output;

                cp(:,1) = this.results.cp(1:n);       %q1
                cp(:,2) = this.results.cp(n+1:2*n);   %q2
                cp(:,3) = this.results.cp(2*n+1:3*n); %q3

                % Construct B-spline trajectory out of control points

                % Knots, triple knots at the end

                knots = [tcp(1),tcp(1),...
                        tcp(1):tcp(end)/length(tcp):tcp(end),...
                        tcp(end),tcp(end)];

                % Least squares fit for the weights of a spline
                s1 = fastBSpline.pspline(knots,2,tcp,cp(:,1),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
                s2 = fastBSpline.pspline(knots,2,tcp,cp(:,2),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
                s3 = fastBSpline.pspline(knots,2,tcp,cp(:,3),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);

                % Reconstructed q-values
                q1      = s1.evalAt(t);         %[1 x length(t) ]
                q2      = s2.evalAt(t);
                q3      = s3.evalAt(t);

                this.data.q_res = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3];

                % Calculate reference velocities
                % Create spline derivative object using fastBSpline function
                s1_d = s1.dx;
                s2_d = s2.dx;
                s3_d = s3.dx;

                q1_d = s1_d.evalAt(t);
                q2_d = s2_d.evalAt(t);
                q3_d = s3_d.evalAt(t);

                this.data.q_d_res = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1_d;q2_d;q3_d];

                % Stop timer
                this.data.runtime   = toc(trun);
                this.data.list      = this.list;
                this.data.params    = this.params;
                this.data.results   = this.results;
                disp(['Total algorithm time: ',num2str(this.data.runtime),' s or ',num2str(this.data.runtime/60),' min']);
            
                
            % Optimize with ESB    
            elseif this.params.noESB == 0
                
                disp('Optimizing with pretension')
                
                % Set pretension postition ranges from legParamsFileName
                if strcmp(this.params.leginput,'Leg_3DoF_design_mono')
                     p_range = getfield(Leg_3DoF_design_mono,'p_range_mono');
                elseif strcmp(this.params.leginput,'Leg_3DoF_design_bi')
                     p_range = getfield(Leg_3DoF_design_bi,'p_range_bi');
                end
                
                % Run optimization with control points [x] = [cp1 cp2 cp3 p]
                % corresponding to [q1 q2 q3 p]
                x0 = [cp_init this.data.p_init]; %initial guess configuration in [3*n + 3, 1] vector corresonding to [q1.. q2.. q3.. p]'

                A   = [];
                b   = [];
                Aeq = [];
                beq = [];

                lb              = zeros(1,tn);
                lb(1:n)         = this.sim.model.leg.params.q_limits(1,1);    %lb q1
                lb(n+1:2*n)     = this.sim.model.leg.params.q_limits(2,1);    %lb q2
                lb(2*n+1:3*n)   = this.sim.model.leg.params.q_limits(3,1);    %lb q3
                lb(3*n+1:3*n+3) = p_range(1:3)';                              %lb p 

                ub              = zeros(1,tn);
                ub(1:n)         = this.sim.model.leg.params.q_limits(1,2);    %ub q1
                ub(n+1:2*n)     = this.sim.model.leg.params.q_limits(2,2);    %ub q2
                ub(2*n+1:3*n)   = this.sim.model.leg.params.q_limits(3,2);    %ub q3
                ub(3*n+1:3*n+3) = p_range(4:6)';                              %ub p  

                % Optimization options
                options = optimoptions( 'fmincon',...           
                                        'Display','iter',...
                                        'Algorithm','interior-point',...
                                        'InitTrustRegionRadius', this.params.InitTrustRegionRadius,... 
                                        'DiffMinChange',this.params.DiffMinChange,...
                                        'DiffMaxChange',this.params.DiffMaxChange,...  
                                        'TypicalX',[cp_init this.data.p_init]);
 
                                       
                % Optimization
                [x, fval, exitflag, output, lambda] = fmincon(@(x)this.ESBjumphigh_obj(x),x0,A,b,Aeq,beq,lb,ub,@(x)this.jumpcon(x),options);

                this.results.cp     = x(1:end-3);
                this.results.p      = x(end-2:end);
                this.results.f      = fval;
                this.results.info   = output;

                cp(:,1) = this.results.cp(1:n);       %q1
                cp(:,2) = this.results.cp(n+1:2*n);   %q2
                cp(:,3) = this.results.cp(2*n+1:3*n); %q3

                % Construct B-spline trajectory out of control points

                % Knots, triple knots at the end

                knots = [tcp(1),tcp(1),...
                        tcp(1):tcp(end)/length(tcp):tcp(end),...
                        tcp(end),tcp(end)];

                % Least squares fit for the weights of a spline
                s1 = fastBSpline.pspline(knots,2,tcp,cp(:,1),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
                s2 = fastBSpline.pspline(knots,2,tcp,cp(:,2),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
                s3 = fastBSpline.pspline(knots,2,tcp,cp(:,3),0.1); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);

                % Reconstructed q-values
                q1      = s1.evalAt(t);         %[1 x length(t) ]
                q2      = s2.evalAt(t);
                q3      = s3.evalAt(t);

                this.data.q_res = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3];

                % Calculate reference velocities
                % Create spline derivative object using fastBSpline function
                s1_d = s1.dx;
                s2_d = s2.dx;
                s3_d = s3.dx;

                q1_d = s1_d.evalAt(t);
                q2_d = s2_d.evalAt(t);
                q3_d = s3_d.evalAt(t);

                this.data.q_d_res = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1_d;q2_d;q3_d];

                % Stop timer
                this.data.runtime   = toc(trun);
                this.data.list      = this.list;
                this.data.params    = this.params;
                this.data.results   = this.results;
                disp(['Total algorithm time: ',num2str(this.data.runtime),' s or ',num2str(this.data.runtime/60),' min']);
            end
        end
        
        %__________________________________________________________________
        % Constraint function
        function [g, geq] = jumpcon(this,x)     
            g = [];
            geq = [];
        end
       
        %__________________________________________________________________
        % Objective function without ESB
        function f = jumphigh_obj(this,x)
            % tn variables optimization
            % Computation of objective function
            % this.data.x = [this.data.x x];
            
            % Assignment of design variables
            Ts  = this.sim.params.Ts;
            n   = this.params.n;
            
            cp(:,1) = x(1:n);       %q1
            cp(:,2) = x(n+1:2*n);   %q2
            cp(:,3) = x(2*n+1:3*n); %q3
            
            this.list.cp = [this.list.cp cp];
            
            % Construct B-spline trajectory out of control points
            
            % Knots, triple knots at the end
            
            t   = this.params.t;
            tcp = this.params.tcp;
            
            knots = [tcp(1),tcp(1),...
                    tcp(1):tcp(end)/length(tcp):tcp(end),...
                    tcp(end),tcp(end)];
            
            % Least squares fit for the weights of a spline
            s1 = fastBSpline.pspline(knots,2,tcp,cp(:,1),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s2 = fastBSpline.pspline(knots,2,tcp,cp(:,2),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s3 = fastBSpline.pspline(knots,2,tcp,cp(:,3),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            
            % Reconstructed q-values
            q1      = s1.evalAt(t);        %[1 x length(t) ]
            q2      = s2.evalAt(t);
            q3      = s3.evalAt(t);
            
            this.data.q_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3]; %6xlength(t)
            
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
                
                
                % Calculate reference velocities
                % Create spline derivative object using fastBSpline function
                s1_d = s1.dx;
                s2_d = s2.dx;
                s3_d = s3.dx;
                
                q1_d = s1_d.evalAt(t);
                q2_d = s2_d.evalAt(t);
                q3_d = s3_d.evalAt(t);
                
                this.data.q_d_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1_d;q2_d;q3_d]; %6xlength(t)
                
                % Run simulation with new q_ref
                this.sim.model.ref.random.q_ref     = this.data.q_rec;
                this.sim.model.ref.random.q_d_ref   = this.data.q_d_rec;

                % Set new initial state
                this.sim.model.setInitialStates( this.sim.model.ref.random.q_ref(:,1), this.sim.model.ref.random.q_d_ref(:,1) )

                % Use new reference for simulation
                this.sim.model.ref.use_random       = 1;
                
                % Run simulation and notice simulation error
                ME = [];
                try
                    this.sim.run(0); 
                    
                % Skip iteration if a joint limit (or other) error still occurs
                catch ME
                end
                if ~isempty(ME)
                    disp(ME.message)
                    f = NaN;
                    ('\n');disp(['Error in simulation, ignoring function evaluation: f = NaN , for ', num2str(length(this.list.f)+1)]);('\n');
                else
                    
                    % Check if leg has fallen over
                    x = this.sim.data.xlist;
                    theta1_all = x(:,21);
                    if (max(theta1_all) > pi/2 || min(theta1_all) < -pi/2)
                        fprintf('\n');disp('Leg has fallen over: f = NaN');fprintf('\n');
                        f = NaN;
                    else
                        
                        
                        % IK, return tau active and maximum CoM-y and mean CoM-x
                        [tau_IK,CoM_xh,CoM_xm, CoM_xf, CoM_y, tlength, RM] = this.Calc_IK();
                        
                        % Performance index CoM height
                        J_high = this.params.c_high*CoM_y^2;
                        fprintf('\n');disp(['J_high = ',num2str(J_high)]);
                        
                        % Stability function 
                        J_stability =   this.params.c_xh * ( CoM_xh )^2 +...
                                        this.params.c_xm * ( CoM_xm )^2 +...
                                        this.params.c_RM * (   RM   )^2;

                        disp(['c * CoM_xh^2 = ',num2str(this.params.c_xh * ( CoM_xh )^2)]) 
                        disp(['c * CoM_xm^2 = ',num2str(this.params.c_xm * ( CoM_xm )^2)])      
                        disp(['c *   RM  ^2 = ',num2str(this.params.c_RM * ( RM )^2)])  
                        disp(['J_stability = ',num2str(J_stability)]);                                   
                        
                        % Penalty function tau
                        
                        % Ignore evaluation if max active torque >= 204
                        if max(max(abs(tau_IK))) >= 204
                            fprintf('\n');disp('Torque limit violated: f = NaN');fprintf('\n');
                            f = NaN;
                        else
                            
                            J_torque = this.params.c_torq * norm(tau_IK)^2;  
                            disp(['J_torque = ',num2str(J_torque)]);

                            % Energy function
                            E_final     = this.sim.data.E(tlength);
                            J_energy    = this.params.c_ener*E_final^2 + ...
                                          this.params.c_y_ref*abs(this.params.CoM_y_ref - CoM_y);
                            disp(['J_energy = ',num2str(J_energy)]);
                            disp(['E = ',num2str(E_final)]);
%                             disp(['y_refdiff = ',num2str(abs(this.params.CoM_y_ref - CoM_y))]);

                            % Objective function

                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            f = -J_high + J_torque + J_stability;
    %                         f = J_energy + J_torque + J_stability;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                            fprintf('\n');disp(['Evaluation succeeded: f = ',num2str(f)]);
                            disp(['Evaluation ',num2str(length(this.list.f)+1)]);('\n');fprintf('\n');
                            disp(['Final CoM x coordinate = ',num2str(CoM_xf)]);
                            disp(['Max CoM height = ',num2str(CoM_y)]);fprintf('\n');

                            % Add function values to list
                            this.list.q_rec         = [this.list.q_rec this.data.q_rec];
                            this.list.J_high        = [this.list.J_high J_high];
                            this.list.J_energy      = [this.list.J_energy J_energy];
                            this.list.J_stability   = [this.list.J_stability J_stability];
                            this.list.J_torque      = [this.list.J_torque J_torque];
                            this.list.f             = [this.list.f f];
                            this.list.E             = [this.list.E E_final];
                            this.list.CoM_y         = [this.list.CoM_y CoM_y];
                        end
                    end
                end
            end
        end
                
        %__________________________________________________________________
        % Objective function with ESB
        function f = ESBjumphigh_obj(this,x)
            % tn variables optimization
            % Computation of objective function
            % this.data.x = [this.data.x x];
            
            % Assignment of design variables
            Ts  = this.sim.params.Ts;
            n   = this.params.n;
            
            cp(:,1)         = x(1:n);               %q1
            cp(:,2)         = x(n+1:2*n);           %q2
            cp(:,3)         = x(2*n+1:3*n);         %q3
            p               = x(3*n+1:3*n+3);       %p vector
            
            this.list.cp    = [this.list.cp cp];
            this.list.p     = [this.list.p p];
            
            % Construct B-spline trajectory out of control points
            
            % Knots, triple knots at the end
            
            t   = this.params.t;
            tcp = this.params.tcp;
            
            knots = [tcp(1),tcp(1),...
                    tcp(1):tcp(end)/length(tcp):tcp(end),...
                    tcp(end),tcp(end)];
            
            % Least squares fit for the weights of a spline
            s1 = fastBSpline.pspline(knots,2,tcp,cp(:,1),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s2 = fastBSpline.pspline(knots,2,tcp,cp(:,2),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s3 = fastBSpline.pspline(knots,2,tcp,cp(:,3),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            
            % Reconstructed q-values
            q1      = s1.evalAt(t);        %[1 x length(t) ]
            q2      = s2.evalAt(t);
            q3      = s3.evalAt(t);
            
            this.data.q_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3]; %6xlength(t)
            
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
                
                
                % Calculate reference velocities
                % Create spline derivative object using fastBSpline function
                s1_d = s1.dx;
                s2_d = s2.dx;
                s3_d = s3.dx;
                
                q1_d = s1_d.evalAt(t);
                q2_d = s2_d.evalAt(t);
                q3_d = s3_d.evalAt(t);
                
                this.data.q_d_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1_d;q2_d;q3_d]; %6xlength(t)
                
                % Run simulation with new q_ref
                this.sim.model.ref.random.q_ref     = this.data.q_rec;
                this.sim.model.ref.random.q_d_ref   = this.data.q_d_rec;
                
                % Set new pretension position
                this.sim.model.p = p;
                
                disp(['p = ',num2str(this.sim.model.p)])
                
                % Set new initial state
                this.sim.model.setInitialStates( this.sim.model.ref.random.q_ref(:,1), this.sim.model.ref.random.q_d_ref(:,1) )
               
                % Use new reference for simulation
                this.sim.model.ref.use_random = 1;
                
                % Run simulatione and notice simulation error
                ME = [];
                try
                    this.sim.run(0); 
                    
                % Skip iteration if a joint limit, p limit or other error still occurs
                catch ME
                end
                if ~isempty(ME)
                    f = NaN;
                    ('\n');disp(['Error in simulation, ignoring function evaluation: f = NaN , for ', num2str(length(this.list.f)+1)]);('\n');
                else
                    
                    % Check if leg has fallen over
                    x = this.sim.data.xlist;
                    theta1_all = x(:,21);
                    if (max(theta1_all) > pi/2 || min(theta1_all) < -pi/2)
                        fprintf('\n');disp('Leg has fallen over: f = NaN');fprintf('\n');
                        f = NaN;
                    else
                        
                        
                        % IK, return tau active and maximum CoM-y and mean CoM-x
                        [tau_IK,CoM_xh,CoM_xm, CoM_xf, CoM_y, tlength, RM] = this.Calc_IK();
                        
                        % Performance index CoM height
                        J_high = this.params.c_high*CoM_y^2;
                        fprintf('\n');disp(['J_high = ',num2str(J_high)]);
                        
                        % Stability function CoM
                        J_stability =   this.params.c_xh * ( CoM_xh )^2 +...
                                        this.params.c_xm * ( CoM_xm )^2 +...
                                        this.params.c_RM * (   RM   )^2;
                        
                        disp(['c * CoM_xh = ',num2str(this.params.c_xh * ( CoM_xh )^2)]) 
                        disp(['c * CoM_xm = ',num2str(this.params.c_xm * ( CoM_xm )^2)])         
                        disp(['c *   RM  ^2 = ',num2str(this.params.c_RM * ( RM )^2)])  
                        disp(['J_stability = ',num2str(J_stability)]);                                   

                        disp(['J_stability = ',num2str(J_stability)]);                                   
                        
                        % Penalty function tau
                        
                        % Ignore evaluation if max active torque >= 204
                        if max(max(abs(tau_IK))) >= 204
                            fprintf('\n');disp('Torque limit violated: f = NaN');fprintf('\n');
                            f = NaN;
                        else
                            
                            J_torque = this.params.c_torq * norm(tau_IK)^2;  
                            disp(['J_torque = ',num2str(J_torque)]);

                            % Energy function
                            E_final     = this.sim.data.E(tlength);
                            J_energy    = this.params.c_ener*E_final^2 + ...
                                          this.params.c_y_ref*abs(this.params.CoM_y_ref - CoM_y);
                            disp(['J_energy = ',num2str(J_energy)]);

                            % Objective function

                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            f = -J_high + J_torque + J_stability;
%                             f = J_energy + J_torque + J_stability;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                            fprintf('\n');disp(['Evaluation succeeded: f = ',num2str(f)]);
                            disp(['Evaluation ',num2str(length(this.list.f)+1)]);('\n');fprintf('\n');
                            disp(['Final CoM x coordinate = ',num2str(CoM_xf)]);
                            disp(['Max CoM height = ',num2str(CoM_y)]);fprintf('\n');

                            % Add function values to list
                            this.list.q_rec         = [this.list.q_rec this.data.q_rec];
                            this.list.J_high        = [this.list.J_high J_high];
                            this.list.J_energy      = [this.list.J_energy J_energy];
                            this.list.J_stability   = [this.list.J_stability J_stability];
                            this.list.J_torque      = [this.list.J_torque J_torque];
                            this.list.f             = [this.list.f f];
                            this.list.E             = [this.list.E E_final];
                            this.list.CoM_y         = [this.list.CoM_y CoM_y];
                        end
                    end
                end
            end
        end
        
        %__________________________________________________________________
        % Inverse Kinematics
            function [tau_IK,CoM_xh,CoM_xm, CoM_xf, CoM_y, tlength, RM] = Calc_IK(this)
                
                % Clear leg state lists
                this.list.q_leg         = [];
                this.list.q_leg_d       = []; 
                
                % Get simulation data
                tau = this.sim.data.tau';
                
                % Timesteps
                x = this.sim.data.xlist;
                
                % Check state list for zero row and cut off time
                cutoff = find(~any(x, 2));
                if isempty(cutoff) ==1
                    t = this.params.t;
                else
                    t = this.params.t(1:cutoff(1)-1);
                end
                
                % Inverse Kinematics
                
                % Preallocate
                qp_dd_t_act = zeros(length(t),6);
                
                % Prepare simulation ground forces storage
                this.data.F_GRF = [];
                
                for k = 1:length(t)
                    % State and input
                    % Retrieve x(t) from all states sim.data.xlist made during simulation
                    x       = this.sim.data.xlist(k,:); 
                    
                    q_leg   = x(18+1:18+6)';       
                    q_leg_d = x(18+7:18+12)';                          
                    
                    u(:,k)  = [tau(k,1); tau(k,2); tau(k,3)];
                    
                    % No external forces
                    Fe = zeros(3,1);
                    
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
                    
                    % Store simulation ground forces
                    this.data.F_GRF = [this.data.F_GRF F_GRF];
                    
                    % Make list of all simulations ground forces
                    this.list.F_GRF = [this.list.F_GRF F_GRF];
                    
                    x       = [q_leg;q_leg_d];
                    
                    % State evaluation to obtain active joint accelerations
                    %x_d    = [q_d; q_dd];
                    x_d     = this.sim.model.leg.dx(t(k), x, u(:,k), F_GRF, Fe); 
                    
                    % Active joint accelerations
                    qa_dd   = x_d(10:12);
                    
                    % Inverse formulation
                    
                    % M-matrices with  M = [Mpp Mpa ; Map Maa]
                    M  = this.sim.model.leg.calc_M(q_leg'); 
                    Mpp = M(1:3,1:3);
                    Mpa = M(1:3,4:6);
                    Map = M(4:6,1:3);
                    Maa = M(4:6,4:6);
                    
                    % New formulation Ma (to be inverted)
                    Ma=[Mpp zeros(3,3); Map -diag(ones(1,3))];
                    
                    % New formulation Mb (rhs)
                    Mb = [Mpa;Maa];
                    
                    % Other terms
                    G       = this.sim.model.leg.calc_G(q_leg);
                    C       = this.sim.model.leg.calc_C(q_leg,q_leg_d); 
                    J_GRF   = this.sim.model.leg.calc_J_GRF(q_leg);
                    D       = this.sim.model.leg.params.d;
                    
                    % Inverse formulation
                    qp_dd_t_act(k,:) =...
                        Ma \ ( -Mb * qa_dd + G - D .* q_leg_d - C * q_leg + J_GRF' * F_GRF );
                    
                    this.list.q_leg     = [this.list.q_leg q_leg];
                    this.list.q_leg_d   = [this.list.q_leg_d q_leg_d];
                end
                
                % Active torque - ESB torque
                t_act  = qp_dd_t_act(:,4:6);  
                tau_IK = t_act - this.sim.data.tau_p(:,1:length(t_act))';
                
                % Get CoM and body positions and velocities
                x       = this.sim.data.xlist;
                q_leg   = x(:,18+1:18+6)';       % 6xN
                q_leg_d = x(:,18+7:18+12)';
                
                % Preallocate rotational moments matrix
                L       = zeros(length(t),3);
                
                for k = 1:length(t)
                    [CoM_x(k), CoM_y(k)] = this.sim.model.leg.calc_CoM(q_leg(:,k));
                    [CoM_x_d(k), CoM_y_d(k)] = this.sim.model.leg.calc_CoM_vel(q_leg(:,k),q_leg_d(:,k));
                    [fwdKin]             = this.sim.model.leg.calc_fwdKin(q_leg(:,k));
                    [fwdKin_vel]         = this.sim.model.leg.calc_fwdKin_vel(q_leg(:,k),q_leg_d(:,k));
                    
                    % Body positions
                    x1    = fwdKin(1);
                    y1    = fwdKin(2);
                    x2    = fwdKin(4);
                    y2    = fwdKin(5);
                    x3    = fwdKin(7);
                    y3    = fwdKin(8);
                    x4    = fwdKin(10);
                    y4    = fwdKin(11);
                    
                    % Body velocities
                    x1_d  = fwdKin_vel(1);
                    y1_d  = fwdKin_vel(2);
                    x2_d  = fwdKin_vel(4);
                    y2_d  = fwdKin_vel(5);
                    x3_d  = fwdKin_vel(7);
                    y3_d  = fwdKin_vel(8);
                    x4_d  = fwdKin_vel(10);
                    y4_d  = fwdKin_vel(11);
                    
                    % Mass
                    Mtot =  this.sim.model.leg.params.m1 + this.sim.model.leg.params.m2 +...
                            this.sim.model.leg.params.m3 + this.sim.model.leg.params.m4;
                    
                    % Relative positions
                    % r_i * m_i * v_i
                    R1 = cross(([x1 y1 0] - [CoM_x(k) CoM_y(k) 0]), this.sim.model.leg.params.m1 * ([x1_d; y1_d; 0] - [CoM_x_d(k); CoM_y_d(k); 0]));
                    R2 = cross(([x2 y2 0] - [CoM_x(k) CoM_y(k) 0]), this.sim.model.leg.params.m2 * ([x2_d; y2_d; 0] - [CoM_x_d(k); CoM_y_d(k); 0]));
                    R3 = cross(([x3 y3 0] - [CoM_x(k) CoM_y(k) 0]), this.sim.model.leg.params.m3 * ([x3_d; y3_d; 0] - [CoM_x_d(k); CoM_y_d(k); 0]));
                    R4 = cross(([x4 y4 0] - [CoM_x(k) CoM_y(k) 0]), this.sim.model.leg.params.m4 * ([x4_d; y4_d; 0] - [CoM_x_d(k); CoM_y_d(k); 0]));

                    % Rotational momentum (all)
                    % R * M * V + sum_i (r_i * m_i * v_i )
                    L(k,:) = cross([CoM_x(k) CoM_y(k) 0], Mtot * [CoM_x_d(k); CoM_y_d(k); 0]) + (R1+R2+R3+R4);
                end

                % Largest y coordinate CoM
                this.data.CoM_y = CoM_y(:);
                [~,i_CoM_y_max] = find(CoM_y == max(CoM_y));
               
                CoM_y           = max(CoM_y);
                
                % Matching x coordinate CoM (matches largest y coordinate CoM)
                % with respect to initial x coordinate CoM
                CoM_xh = (CoM_x(i_CoM_y_max)-CoM_x(1));
                
                % Mean x coordinate from start to heighest point
                % with respect to initial x coordinate CoM
                CoM_xm = mean(abs(CoM_x - ones(1,length(CoM_x))*CoM_x(1)));
                
                % Final x coordinate
                CoM_xf = CoM_x(end);
                
                % Rotational momentum at heighest point
                RM = L(i_CoM_y_max,3);
                
                %Simulation time
                tlength = length(t);

            end
        %__________________________________________________________________________
        function plot(this) 
            % Plots optimization results 
            paperMode = confirm('Generate plots in paper mode? [y/N]', 0);
            savePlots = confirm('Save plots as PDF? [y/N]', 0);
            
             % Get plot size depending on paperMode
            if (paperMode)
                figSize = this.plots.sizePaperMode;
            else
                figSize = this.plots.sizeNormal;
            end
            
            % Plot directory
            plotPath    = 'plots/';
            if (~exist(plotPath, 'dir'))
                   mkdir(plotPath); 
            end
            % Get some plotting parameters
%             filtIgnoreTime          = this.plots.filtIgnoreTime;
%             yAxisIgnoreTime         = this.plots.yAxisIgnoreTime;
            showTitleInPaperMode    = this.plots.showTitleInPaperMode;
            C                       = this.plots.C;         % Plot colours
            C_light                 = this.plots.C_light;   % Light plot colours
            
            n       = this.params.n;
            N       = length(this.params.t);
            x       = this.sim.data.xlist; %[N x 30]
            q_leg   = x(:,18+1:18+6)';       % 6xN
            q_leg_d = x(:,18+7:18+12)';    % 6xN 
            
            
            figure
            plot(this.params.t, this.data.q_init(:,4)',this.params.t,this.data.q_res(4,:),'--',this.params.t,q_leg(4,:))
            title('q_1');legend('Initial reference','Solution reference','Actual trajectory');xlabel('Time [s]');ylabel('Angle [rad]');
            
            figure
            plot(this.params.t, this.data.q_init(:,5)',this.params.t,this.data.q_res(5,:),'--',this.params.t,q_leg(5,:))
            title('q_2');legend('Initial reference','Solution reference','Actual trajectory');xlabel('Time [s]');ylabel('Angle [rad]');
            
            figure
            plot(this.params.t, this.data.q_init(:,6)',this.params.t,this.data.q_res(6,:),'--',this.params.t,q_leg(6,:))
            title('q_3');legend('Initial reference','Solution reference','Actual trajectory');xlabel('Time [s]');ylabel('Angle [rad]');
            
            
            % Plot all q1 references
            figure
            resizeFig(gcf, figSize(1), figSize(2));
            plot(this.params.t,this.list.q_rec(4,1:length(this.params.t)),'g')
            hold on
            plot(this.params.t,this.data.q_res(4,:),'m')
            for k = 1:length(this.list.f)
                greenfade = [0 1/length(this.list.f)*k 0];
                plot(this.params.t,this.list.q_rec(4,(k-1)*length(this.params.t)+1:k*length(this.params.t)),'Color',greenfade)
            end
            plot(this.params.t,this.data.q_res(4,:),'m','Linewidth',1.5)
            hold off
            setYAxis;
            paperModeLegend(    paperMode, ...
                                {'$q_{1,initial}$','$q_{1,final}$'},   ...
                                {'q_1 final'}    );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '$q_1$ [rad]'}, {'t [s]', '[rad]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Trajectories $q_1$',  ...
                                'Trajectories $q_1$'   );
            paperSave(savePlots, [plotPath 'q1_traj.pdf']);
            
            % Plot all q2 references
            figure
            resizeFig(gcf, figSize(1), figSize(2));
            plot(this.params.t,this.list.q_rec(5,1:length(this.params.t)),'g')
            hold on
            plot(this.params.t,this.data.q_res(5,:),'m')
            for k = 1:length(this.list.f)
                greenfade = [0 1/length(this.list.f)*k 0];
                plot(this.params.t,this.list.q_rec(5,(k-1)*length(this.params.t)+1:k*length(this.params.t)),'Color',greenfade)
            end
            plot(this.params.t,this.data.q_res(5,:),'m','Linewidth',1.5)
            hold off
            setYAxis;
            paperModeLegend(    paperMode, ...
                                {'$q_{2,initial}$','$q_{2,final}$'},   ...
                                {'q_2 initial','q_2 final'}    );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '$q_2$ [rad]'}, {'t [s]', '[rad]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Trajectories $q_2$',  ...
                                'Trajectories $q_2$'   );
            paperSave(savePlots, [plotPath 'q2_traj.pdf']);           
            
            % Plot all q3 references
            figure
            resizeFig(gcf, figSize(1), figSize(2));
            plot(this.params.t,this.list.q_rec(6,1:length(this.params.t)),'g')
            hold on
            plot(this.params.t,this.data.q_res(6,:),'m')
            for k = 1:length(this.list.f)
                greenfade = [0 1/length(this.list.f)*k 0];
                plot(this.params.t,this.list.q_rec(6,(k-1)*length(this.params.t)+1:k*length(this.params.t)),'Color',greenfade)
            end
            plot(this.params.t,this.data.q_res(6,:),'m','Linewidth',1.5)
            hold off
            setYAxis;
            paperModeLegend(    paperMode, ...
                                {'$q_{3,initial}$','$q_{3,final}$'},   ...
                                {'q_3 initial','q_3 final'}    );
            paperModeAxisLabels(paperMode, {'$t$ [s]', '$q_3$ [rad]'}, {'t [s]', '[rad]'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Trajectories $q_3$',  ...
                                'Trajectories $q_3$'   );
            paperSave(savePlots, [plotPath 'q3_traj.pdf']);  
            
            % Plot evolution of criteria
            xf =length(this.list.f);
            figure 
            resizeFig(gcf, figSize(1), 500);
            
            subplot(3,1,1)
            plot(1:length(this.list.J_high),this.list.J_high,'.');
            axis([0 xf 350 500])
            paperModeAxisLabels(paperMode, {'Objective function evaluation','Performance criterion'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Performance criterion (high jump)',  ...
                                'Performance criterion (high jump)'   );        
                            
            subplot(3,1,2)
            plot(1:length(this.list.J_stability),this.list.J_stability,'.');
            axis([0 xf 0 15])
            paperModeAxisLabels(paperMode, {'Objective function evaluation','Stability criterion'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Stability criterion',  ...
                                'Stability criterion'   );       
                            
            subplot(3,1,3)
            plot(1:length(this.list.J_torque),this.list.J_torque,'.');
            axis([0 xf 0 450])
            paperModeAxisLabels(paperMode, {'Objective function evaluation','Torque criterion'});
            paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Torque criterion',  ...
                                'Torque criterion'   );       
            paperSave(savePlots, [plotPath 'crit_high.pdf']);  
                            
            % Plot evolution control points
            C = this.list.cp;
            n = this.params.n;
            cpq1 = C(:,1:3:end);
            cpq2 = C(:,2:3:end);
            cpq3 = C(:,3:3:end);
     
            s = size(cpq3); s=1:1:s(2);
            
            figure
            resizeFig(gcf, figSize(1), 500);
            for k=1:n        
               subplot(3,1,1);plot(s,cpq1(k,:),'.'); axis([1 s(end) min(min(cpq1))-0.3 max(max(cpq1))+0.3]);...
               paperModeAxisLabels(paperMode, {'Objective function evaluation','$q_1$ [rad]'});
               paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Control Points $q_1$',  ...
                                'Control Points q_1'   ); 
               hold on
               subplot(3,1,2);plot(s,cpq2(k,:),'.');  axis([1 s(end) min(min(cpq2))-0.3 max(max(cpq2))+0.3]);...
               paperModeAxisLabels(paperMode, {'Objective function evaluation','$q_2$ [rad]'});
               paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Control Points $q_2$',  ...
                                'Control Points q_2'   );                hold on
               subplot(3,1,3);plot(s,cpq3(k,:),'.');  axis([1 s(end) min(min(cpq3))-0.3 max(max(cpq3))+0.3]);...
               paperModeAxisLabels(paperMode, {'Objective function evaluation','$q_3$ [rad]'});
               paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                'Control Points $q_3$',  ...
                                'Control Points q_3'   );                hold on
            end
            hold off
            
            % Plot evolution pretensions if optimized
            if this.params.noESB == 0
                p1 = this.data.list.p(:,1:3:end);
                p2 = this.data.list.p(:,2:3:end);
                p3 = this.data.list.p(:,3:3:end);
                
                figure
                resizeFig(gcf, figSize(1), figSize(2));
                plot(s,p1,s,p2,s,p3);title('Pretension positions evolution');...
                axis([1 s(end) min(min(this.data.list.p))-0.01 max(max(this.data.list.p))+0.01]);...
                paperModeLegend(    paperMode, ...
                                {'$p_1$','$p_2$','$p_3$'},   ...
                                {'p_1','p_2','p_3'}    );
                paperModeAxisLabels(paperMode, {'Objective function evaluation','Pretension position [m]'});
                paperModeTitle(     showTitleInPaperMode, paperMode, ...
                                    'Pretension positions',  ...
                                    'Pretension positions'   ); 
            end
            
            % Plot latest ground forces
            F_GRF_all = zeros(4,length(this.params.t));
            for k = 1:length(this.data.F_GRF)
               F_GRF_all(:,k) = this.data.F_GRF(:,k);
            end
            figure
            plot(this.params.t,F_GRF_all,'Linewidth',2);title('Ground Forces latest simulation');...
                xlabel('Time [s]'),ylabel('Ground Force [N]');legend('Ankle x','Ankle y','Toe x','Toe y');
            
        end

        %__________________________________________________________________
        %
        function plotlatest(this) 
            figure
            plot(this.params.t, this.data.q_init(:,4)',this.params.t,this.data.q_rec(4,:),'--')
            title('q_1');legend('Original','Latest');xlabel('s');ylabel('rad');

            figure
            plot(this.params.t, this.data.q_init(:,5)',this.params.t,this.data.q_rec(5,:),'--')
            title('q_2');legend('Original','Latest');xlabel('s');ylabel('rad');

            figure
            plot(this.params.t, this.data.q_init(:,6)',this.params.t,this.data.q_rec(6,:),'--')
            title('q_3');legend('Original','Latest');xlabel('s');ylabel('rad');
            
             % Plot evolution control points
            n       = this.params.n;
            cpq1    = this.list.cp(:,1:3:end);
            cpq2    = this.list.cp(:,2:3:end);
            cpq3    = this.list.cp(:,3:3:end);
            s       = size(cpq1);s=1:1:s(2);
            
            for k=1:n
               hold on
               subplot(3,1,1);plot(s,cpq1(k,:),'-.');title('Control Points q_1'); xlim([1 s(end)]);
               hold on
               subplot(3,1,2);plot(s,cpq2(k,:),'-.');title('Control Points q_2');  xlim([1 s(end)]);
               hold on
               subplot(3,1,3);plot(s,cpq3(k,:),'-.');title('Control Points q_3');  xlim([1 s(end)]);
               hold on
            end
            hold off
        
        end
        %__________________________________________________________________    
        
        function q_init_builder(this)
        % Can be used to construct and save a simple initial trajectory.
        % This function is to be used individually. The algorithm loads
        % the .mat file created by this function as its initial trajectory.
            
            Ts      = this.sim.params.Ts;
            tspan   = this.sim.params.tspan;
            t       = 0:Ts:tspan(2);
            
                % Calculate references for joints
                % Leg: q = [x1, y1, theta, q1, q2, q3]
                q_ref           = zeros(6,length(t));
                %q_init
                q_ref(:,1)      = [0; 0; 0; -0.6; 1.5; -1.2];
                
                % Timing stages, time span in seconds
                 stg1 = 0.1; 

            for k = 1:length(t)
                % Stage 1
                if (0<k*Ts) && (k*Ts<=stg1)
                    q_ref(4,k)  = -0.6;
                    q_ref(5,k)  = 1.5;
                    q_ref(6,k)  = -1.2;
                end

                % Stage 2:  push off
                if (stg1<k*Ts) 

                    q_ref(4,k)  = -0.6;
                    q_ref(5,k)  = 1.4;
                    q_ref(6,k)  = -1.0;
                end
            end
            
            % Smoothen q_ref from step to spline
            siteres = 50;
            lessdata4 = q_ref(4,1:siteres:end);
            lessdata5 = q_ref(5,1:siteres:end);
            lessdata6 = q_ref(6,1:siteres:end);

            sites   = 0:Ts*siteres:t(end);
            x       = linspace(0,1,length(t));
            q_init  = zeros(6,length(t));

            y4  = lessdata4; y5 = lessdata5; y6 = lessdata6;

            cs4 = pchip(sites,y4);
            cs5 = pchip(sites,y5);
            cs6 = pchip(sites,y6);


            q_init(4,:) = ppval(x,cs4);
            q_init(5,:) = ppval(x,cs5);
            q_init(6,:) = ppval(x,cs6);
            
            % Write q_init to .mat file
            q_init = q_init';
            save('q_init','q_init')
        end
         %__________________________________________________________________    
        
        function simulate_q_init(this)
            
            % Load q_init .mat file
            load('q_init','q_init')
            
            cp_init(:,1) = q_init(1:this.params.cpres:end,4); % q1
            cp_init(:,2) = q_init(1:this.params.cpres:end,5); % q2
            cp_init(:,3) = q_init(1:this.params.cpres:end,6); % q3
            
            this.data.cp_init = cp_init;
            % Construct B-spline trajectory out of control points
            
            % Knots, triple knots at the end
            
            Ts      = this.sim.params.Ts;
            tspan   = this.sim.params.tspan;
            t       = 0:Ts:tspan(2);
            tcp     = t(1:this.params.cpres:end);
            
            knots = [tcp(1),tcp(1),...
                    tcp(1):tcp(end)/length(tcp):tcp(end),...
                    tcp(end),tcp(end)];
            
            % Least squares fit for the weights of a spline
            s1 = fastBSpline.pspline(knots,2,tcp,cp_init(:,1),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s2 = fastBSpline.pspline(knots,2,tcp,cp_init(:,2),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            s3 = fastBSpline.pspline(knots,2,tcp,cp_init(:,3),0.01); %fastBSpline.pspline(knots,order,x,y,smoothing penalty);
            
            % Reconstructed q-values
            q1      = s1.evalAt(t);        %[1 x length(t) ]
            q2      = s2.evalAt(t);
            q3      = s3.evalAt(t);
            
            this.data.q_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1;q2;q3]; %6xlength(t)
            
            % Check the constructed trajectories for q-limits and
            % work around
            
            if (    min(q1) < this.sim.model.leg.params.q_limits(1,1)   )
                i = find(q1 == min(q1));
                fprintf('\n');disp(['Joint q1 min limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            elseif (    max(q1) > this.sim.model.leg.params.q_limits(1,2)    )
                i = find(q1 == max(q1));
                fprintf('\n');disp(['Joint q1 max limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            elseif (    min(q2) < this.sim.model.leg.params.q_limits(2,1) )
                i = find(q2 == min(q2));
                fprintf('\n');disp(['Joint q2 min limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            elseif (    max(q2) > this.sim.model.leg.params.q_limits(2,2)    )
                i = find(q2 == max(q2));
                fprintf('\n');disp(['Joint q2 max limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            elseif (    min(q3) < this.sim.model.leg.params.q_limits(3,1)    )
                i = find(q3 == min(q3));
                fprintf('\n');disp(['Joint q3 min limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            elseif (       max(q3) > this.sim.model.leg.params.q_limits(3,2)    )
                i = find(q3 == max(q3));
                fprintf('\n');disp(['Joint q3 max limit reached at t = ',num2str(i*Ts)]);fprintf('\n');
            else
                
                
                % Calculate reference velocities
                % Create spline derivative object using fastBSpline function
                s1_d = s1.dx;
                s2_d = s2.dx;
                s3_d = s3.dx;
                
                q1_d = s1_d.evalAt(t);
                q2_d = s2_d.evalAt(t);
                q3_d = s3_d.evalAt(t);
                
                this.data.q_d_rec = [zeros(1,length(t));zeros(1,length(t));zeros(1,length(t));q1_d;q2_d;q3_d]; %6xlength(t)
                
                % Run simulation with new q_ref
                this.sim.model.ref.random.q_ref     = this.data.q_rec;
                this.sim.model.ref.random.q_d_ref   = this.data.q_d_rec;

                % Set new initial state
                this.sim.model.p = this.data.p_init;
                this.sim.model.setInitialStates( this.sim.model.ref.random.q_ref(:,1), this.sim.model.ref.random.q_d_ref(:,1) )
                
                % Use new reference for simulation
                this.sim.model.ref.use_random = 1;
                
                % Run simulation
                this.sim.run(1);
            end
        end
        %_____________________________________________________________
        
        function simulate_solution(this)      
%        % Run simulation with q_res (result)
            this.sim.model.ref.use_random =1;
            this.sim.model.ref.random.q_ref = this.data.q_res;   
            this.sim.model.ref.random.q_d_ref = this.data.q_d_res;
            
            % Set initial state
            if this.params.noESB == 0
                this.sim.model.p = this.data.results.p;
            end
            this.sim.model.setInitialStates( this.sim.model.ref.random.q_ref(:,1), this.sim.model.ref.random.q_d_ref(:,1) )
  
            fprintf('\n');disp('Resimulating for found solution');
            this.sim.run(1);
            
            % Show CoM_y
            [~,~,~,~, CoM_y,~] = this.Calc_IK;
            fprintf('\n');disp(['CoM_y = ',num2str(CoM_y),' m']);
            
            % Show cumulative energy consumption
            disp(['Energy consumption = ',num2str(this.sim.data.E(end)),' J'])
            
            % Show p if optimized
            if this.params.noESB == 0
                disp(['p = ',num2str(this.data.results.p),' m']);
            end
            fprintf('\n');disp('Rerun animation with this.sim.animate');     
         end
        %_____________________________________________________________
        function plot_matdata(this)
            
            if (~exist('optimization_data','var')) || (~exist('simulation_data','var'))
                this.data           = evalin('base','optimization_data');
                this.list           = this.data.list;
                this.sim.data       = evalin('base','simulation_data');
                this.results        = this.data.results;
                this.params         = this.data.params;
                this.sim.plot;
                hold on
                this.plot;
            else
                disp('Not all needed data found. Please load .mat file');
            end
            
        end 
        %_____________________________________________________________
        function load_matdata(this)
            if (~exist('optimization_data','var')) || (~exist('simulation_data','var'))
                this.data           = evalin('base','optimization_data');
                this.list           = this.data.list;
                this.sim.data       = evalin('base','simulation_data');
                this.results        = this.data.results;
                this.params         = this.data.params;
            else
                disp('Not all needed data found. Please load .mat file');
            end            
        end
    end %end methods
end %end classdef























