classdef Leg_3DoF_noAct < handle
    
    %__________________________________________________________________
    % Properties
    properties
        leg     % Leg_3DoF object
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF_noAct(legParamsFileName, quiet)
            % Set default arguments
            
            % legParamsFileName
            if (~exist('legParamsFileName', 'var'))
                legParamsFileName = '';
            end
            
            % quiet
            if (~exist('quiet', 'var'))
                quiet = 0;
            end
            
            % Get leg object
            this.leg    = Leg_3DoF(legParamsFileName);
           
            % Set initial leg states from straight leg configuration
            % Leg: q = [x1, y1, theta, q1, q2, q3]
            q_0     = [0; 0.02; 0; 0.0; 0.0; 0.0]; % Leg starts slightly in the air
            q_d_0   = [0; 0; 0; 0; 0; 0];
            this.setInitialStates(q_0, q_d_0);
        end
        
        
        %__________________________________________________________________
        % Set initial states of leg and actuators properly
        function [ ] = setInitialStates( this, q_0, q_d_0 )
            % Set initial leg state
            this.leg.x0     = [q_0; q_d_0];
            
            % Get joints
            q_joints_0      = q_0(4:6);
            q_joints_d_0    = q_d_0(4:6);
        end
        
        
        %__________________________________________________________________
        % State evolution - function to be handed to ode45()
        function [ dx ] = dx_ode( this, t, x )
%             [dx, ~, ~, ~] = this.dx(t,x);
            [dx] = this.dx(t,x);
        end

        
        %__________________________________________________________________
        % State evolution
%         function [ dx, y, u, tau_ref ] = dx( this, t, x )
       function [ dx, y, u] = dx( this, t, x )
            x_leg           = x(:); 

            % Get outputs

            q_leg   = x_leg(1 : this.leg.N/2); %#ok<*NASGU>     
            q_leg_d = x_leg(this.leg.N/2 + 1 : this.leg.N);

            % No Actuation whatsoever
            u_leg = zeros(3,1);
            u       = u_leg;
            
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
            
 
                Fe = [0; 0; 0];

            % Get leg dynamics
            dx_leg  = this.leg.dx(t, x_leg, u_leg, F_GRF, Fe);

            % Assemble state derivatives
             dx      = dx_leg;
            

            
            % Get the leg output
            y_leg   = this.leg.y(t, x_leg);
            
            % Get the total output
            y       = y_leg;

        end  
                     
    end
    
end

