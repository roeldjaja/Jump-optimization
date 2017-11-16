classdef Leg_3DoF < handle
%Leg_3DoF - 3-DoF leg model class
    
    %__________________________________________________________________
    % Private properties
    properties
    end
    
    %__________________________________________________________________
    % Public properties
    properties (SetAccess = public)
        params;     % Parameters struct
        N;          % Number of states N
        M;          % Number of inputs M
        K;          % Number of outputs K
        x0;         % Initial conditions
    end
    
    %__________________________________________________________________
    % Methods
    methods
        
        %__________________________________________________________________
        % Constructor
        function this = Leg_3DoF(paramsName)
            % Default parameters
            if (~exist('paramsName', 'var') || strcmp(paramsName, ''))
                paramsName = 'Leg_3DoF_design_noESB';
            end
            
            % Load parameters
            disp(['Constructing Leg_3DoF with parameters ''' paramsName '''...']);
            this.params = eval(paramsName);
            
            % Set dimensions
            % q = [x1, y1, theta1, q1, q2, q3]
            this.N = 12;    % Number of states N    % x = [q; q_d]
            this.M = 3;     % Number of inputs M    % u = [tau_1, tau_2, tau_3]
            this.K = 12;    % Number of outputs K   % y = x
            
            % Initial conditions
            % q = [x1, y1, theta1, q1, q2, q3]
            q_0     = [0; 0; 0; 0; 0; 0];
            q_d_0   = [0; 0; 0; 0; 0; 0];
            this.x0 = [q_0; q_d_0];                 % x = [q; q_d]
        end
        
        %__________________________________________________________________
        % Evolve dynamics given time t, state x, input u, ground reaction
        % forces F_GRF, and external force Fe
        function x_d = dx(this, t, x, u, F_GRF, Fe) %#ok<*INUSD,*INUSL>
%              q = [x1, y1, theta1, q1, q2, q3];
%              x = [q; q_d];%
            q       = x(1 : this.N/2);
            q_d     = x(this.N/2 + 1 : this.N);
            
            % Check joint limits
            if (    q(4) < this.params.q_limits(1,1)    || ...
                    q(4) > this.params.q_limits(1,2)    )
                error(['Leg has hit joint limit on joint 1: q(4) = ' num2str(q(4)) ' at t = ' num2str(t) '!']);
            end
            if (    q(5) < this.params.q_limits(2,1)    || ...
                    q(5) > this.params.q_limits(2,2)    )
                error(['Leg has hit joint limit on joint 2: q(5) = ' num2str(q(5)) ' at t = ' num2str(t) '!']);
            end
            if (    q(6) < this.params.q_limits(3,1)    || ...
                    q(6) > this.params.q_limits(3,2)    )
                error(['Leg has hit joint limit on joint 3: q(6) = ' num2str(q(6)) ' at t = ' num2str(t) '!']);
            end
            
            % Generalised actuation forces are [0; 0; 0; tau_1; tau_2; tau_3],
            % the first 3 being the non-actuated floating base
            tau = [0; 0; 0; u];
            
            % Get inverted inertia matrix, gravitational force vector,
            % damping vector, Coriolis vector
            Minv    = this.calc_M_inv(q);
            G       = this.calc_G(q);
            D       = this.params.d; % vector so dot-product
            C       = this.calc_C(q, q_d);
            
            % Get Jacobians for GRF and external force on trunk
            J_GRF   = this.calc_J_GRF(q);
            Je      = this.calc_Je(q);
           
            % q_dd = M^-1 * [G(q) + tau - D .* q_d - C * q_d + J_GRF' * F_GRF + Je' * Fe]            
            q_dd    = Minv * (G + tau - D .* q_d - C * q_d + J_GRF' * F_GRF + Je' * Fe);
            
            % Combine to obtain state derivative
            x_d     = [q_d; q_dd];
        end
        
        %__________________________________________________________________
        % Get the output y
        function y = y(this, t, x)
            % y = x
            y = x;
        end
        
        %__________________________________________________________________
        % Calculate the inverse inertia matrix
        function [ M_inv ] = calc_M_inv(this, q)
            % Get the inertia matrix
            M = this.calc_M(q); %#ok<*PROPLC>

            % Calculate the inverse matrix
            M_inv = inv(M);
        end
        
        %__________________________________________________________________
        % Calculate the inertia matrix
        function [ M ] = calc_M(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);

            % Get the Jacobian
            J = this.calc_J(q);

            % Construct the modal inertial matrix
            Mc = diag([m1, m1, J1, m2, m2, J2, m3, m3, J3, m4, m4, J4]);

            % Calculate the inertia matrix
            M = J' * Mc * J;
        end
        
        %__________________________________________________________________
        % Calculates the system Jacobian
        function [ J ] = calc_J(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);

            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);

            % Construct J
            J = [   1, 0, 0,                                                    0,                                          0,                              0; ...
                    0, 1, 0,                                                    0,                                          0,                              0; ...
                    0, 0, 1,                                                    0,                                          0,                              0; ...
                    1, 0, -r2 * ct1q1 + r1 * st1,                               -r2 * ct1q1,                                0,                              0; ...
                    0, 1, -r2 * st1q1 - r1 * ct1,                               -r2 * st1q1,                                0,                              0; ...
                    0, 0, 1,                                                    1,                                          0,                              0; ...
                    1, 0, -r3 * ct1q12 - l2 * ct1q1 + r1 * st1,                 -r3 * ct1q12 - l2 * ct1q1,                  -r3 * ct1q12,                   0; ...
                    0, 1, -r3 * st1q12 - l2 * st1q1 - r1 * ct1,                 -r3 * st1q12 - l2 * st1q1,                  -r3 * st1q12,                   0; ...
                    0, 0, 1,                                                    1,                                          1,                              0; ...
                    1, 0, -r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1 + r1 * st1,  -r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1,   -r4 * ct1q123 - l3 * ct1q12,    -r4 * ct1q123; ...
                    0, 1, -r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1,  -r4 * st1q123 - l3 * st1q12 - l2 * st1q1,   -r4 * st1q123 - l3 * st1q12,    -r4 * st1q123; ...
                    0, 0, 1,                                                    1,                                          1,                              1, ...
                ];
        end
        
        %__________________________________________________________________
        % Calculates the end-effector Jacobian J_GRF
        function [ J_GRF ] = calc_J_GRF(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);
            
            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);
            
            % Construct J_GRF
            J_GRF = [   1, 0, r_heel * st1,     0, 0, 0; ...
                        0, 1, -r_heel * ct1,    0, 0, 0; ...
                        1, 0, -r_toe * st1,     0, 0, 0; ...
                        0, 1, r_toe * ct1,      0, 0, 0; ...
                    ];
        end
        
        %__________________________________________________________________
        % Calculates the end-effector Jacobian Je/J_trunk
        function [ Je ] = calc_Je(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);
            
            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);
            
            % Construct Je
            Je = [  1, 0, -r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1 + r1 * st1,  -r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1,   -r4 * ct1q123 - l3 * ct1q12,    -r4 * ct1q123; ...
                    0, 1, -r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1,  -r4 * st1q123 - l3 * st1q12 - l2 * st1q1,   -r4 * st1q123 - l3 * st1q12,    -r4 * st1q123; ...
                    0, 0, 1,                                                    1,                                          1,                              1; ...
                ];
        end
        
        %__________________________________________________________________
        % [ C ] = calc_C( q )
        % Calculate the Coriolis matrix
        function [ C ] = calc_C( this, q, q_d )
            % Get parameters
            m_1     = this.params.m1;
            m_2     = this.params.m2;
            m_3     = this.params.m3;
            m_4     = this.params.m4;
            J_1     = this.params.J1;
            J_2     = this.params.J2;
            J_3     = this.params.J3;
            J_4     = this.params.J4;
            l_1     = this.params.l1;
            l_2     = this.params.l2;
            l_3     = this.params.l3;
            l_4     = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r_1     = this.params.r1;
            r_2     = this.params.r2;
            r_3     = this.params.r3;
            r_4     = this.params.r4;
            g       = this.params.g;
            
            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x_1         = q(1,:);
            y_1         = q(2,:);
            theta_1     = q(3,:);
            q_1         = q(4,:);
            q_2         = q(5,:);
            q_3         = q(6,:);

            % Get state derivatives
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x_1_d       = q_d(1,:);
            y_1_d       = q_d(2,:);
            theta_1_d   = q_d(3,:);
            q_1_d       = q_d(4,:);
            q_2_d       = q_d(5,:);
            q_3_d       = q_d(6,:);

            % Calculate the Coriolis matrix
            % See coriolis_sym()
            C(1,:) = [0, 0,q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)) + theta_1_d*(m_3*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) + m_2*(r_2*sin(q_1 + theta_1) + r_1*cos(theta_1)) + m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1)) + m_2*r_2*sin(q_1 + theta_1)) + m_4*q_3_d*r_4*sin(q_1 + q_2 + q_3 + theta_1), q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)) + q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1)) + m_2*r_2*sin(q_1 + theta_1)) + theta_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1)) + m_2*r_2*sin(q_1 + theta_1)) + m_4*q_3_d*r_4*sin(q_1 + q_2 + q_3 + theta_1), q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)) + q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)) + theta_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)) + m_4*q_3_d*r_4*sin(q_1 + q_2 + q_3 + theta_1), m_4*q_1_d*r_4*sin(q_1 + q_2 + q_3 + theta_1) + m_4*q_2_d*r_4*sin(q_1 + q_2 + q_3 + theta_1) + m_4*q_3_d*r_4*sin(q_1 + q_2 + q_3 + theta_1) + m_4*r_4*theta_1_d*sin(q_1 + q_2 + q_3 + theta_1)];
            C(2,:) = [0, 0,- theta_1_d*(m_3*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_2*(r_2*cos(q_1 + theta_1) - r_1*sin(theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1))) - q_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)) - q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)) - m_4*q_3_d*r_4*cos(q_1 + q_2 + q_3 + theta_1), - q_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)) - theta_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)) - q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)) - m_4*q_3_d*r_4*cos(q_1 + q_2 + q_3 + theta_1), - q_1_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)) - q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)) - theta_1_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)) - m_4*q_3_d*r_4*cos(q_1 + q_2 + q_3 + theta_1), - m_4*q_1_d*r_4*cos(q_1 + q_2 + q_3 + theta_1) - m_4*q_2_d*r_4*cos(q_1 + q_2 + q_3 + theta_1) - m_4*q_3_d*r_4*cos(q_1 + q_2 + q_3 + theta_1) - m_4*r_4*theta_1_d*cos(q_1 + q_2 + q_3 + theta_1)];
            C(3,:) = [0, 0, q_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1))*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)*(r_2*sin(q_1 + theta_1) + r_1*cos(theta_1)) - m_2*r_2*sin(q_1 + theta_1)*(r_2*cos(q_1 + theta_1) - r_1*sin(theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))), q_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1))*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)*(r_2*sin(q_1 + theta_1) + r_1*cos(theta_1)) - m_2*r_2*sin(q_1 + theta_1)*(r_2*cos(q_1 + theta_1) - r_1*sin(theta_1))) + theta_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1))*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)*(r_2*sin(q_1 + theta_1) + r_1*cos(theta_1)) - m_2*r_2*sin(q_1 + theta_1)*(r_2*cos(q_1 + theta_1) - r_1*sin(theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))), q_1_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + q_2_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))) + theta_1_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))), - q_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - q_2_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - theta_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)))];
            C(4,:) = [0, 0, q_3_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) - theta_1_d*(m_3*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1))*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1)) + m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_2*r_2*cos(q_1 + theta_1)*(r_2*sin(q_1 + theta_1) + r_1*cos(theta_1)) - m_2*r_2*sin(q_1 + theta_1)*(r_2*cos(q_1 + theta_1) - r_1*sin(theta_1))) - q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))), q_3_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) - q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))), q_3_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) - q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))) - q_2_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))) - theta_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))), q_1_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) + q_2_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) + q_3_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) + theta_1_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)))];
            C(5,:) = [0, 0, q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))) - theta_1_d*(m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) + m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + r_3*sin(q_1 + q_2 + theta_1)) - m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + r_3*cos(q_1 + q_2 + theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))), q_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + theta_1_d*(m_4*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) + m_3*r_3*sin(q_1 + q_2 + theta_1)*(l_2*cos(q_1 + theta_1) + r_3*cos(q_1 + q_2 + theta_1)) - m_3*r_3*cos(q_1 + q_2 + theta_1)*(l_2*sin(q_1 + theta_1) + r_3*sin(q_1 + q_2 + theta_1))),-q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))), - q_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - q_2_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - q_3_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - theta_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)))];
            C(6,:) = [0, 0, q_2_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - q_1_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) + theta_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) - r_1*sin(theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + r_1*cos(theta_1) + l_3*sin(q_1 + q_2 + theta_1))), q_2_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) - theta_1_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))) - q_1_d*(m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_2*sin(q_1 + theta_1) + l_3*sin(q_1 + q_2 + theta_1)) - m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_2*cos(q_1 + theta_1) + l_3*cos(q_1 + q_2 + theta_1))), q_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + q_2_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))) + theta_1_d*(m_4*r_4*sin(q_1 + q_2 + q_3 + theta_1)*(r_4*cos(q_1 + q_2 + q_3 + theta_1) + l_3*cos(q_1 + q_2 + theta_1)) - m_4*r_4*cos(q_1 + q_2 + q_3 + theta_1)*(r_4*sin(q_1 + q_2 + q_3 + theta_1) + l_3*sin(q_1 + q_2 + theta_1))), 0];
        end
        
        %__________________________________________________________________
        % [ Fry, CoP ] = calc_GRF_CoP_static(q)
        % Calculate the CoP location (for a static posture) and ground reaction
        % force (GRF) in vertical direction (since statically horizontal force is
        % zero)
        function [ Fry, CoP ] = calc_GRF_CoP_static(this, q)
            % Get parameters and calculate total system mass
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            r1      = this.params.r1;
            g       = this.params.g;
            m_tot   = m1 + m2 + m3 + m4;
            
            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1,:);
            y1      = q(2,:);
            theta1  = q(3,:);
            q1      = q(4,:);
            q2      = q(5,:);
            q3      = q(6,:);
            
            % Calculate vertical ground reaction force
            Fry = m_tot * this.params.g;
            
            % Calculate generalised gravitation vector
            G = this.calc_G(q);

            % Calculate CoP location
            % q = [x1, y1, theta, q1, q2, q3]
            % G(4,:) = Torque at ankle
            % The CoP is defined in the global frame using x1 and r1
            CoP     = (-G(4,:)' ./ Fry)' + x1 - r1;
            Fry     = Fry * ones(size(CoP));
        end
        
        %__________________________________________________________________
        % [ Frx, Fry, CoP ] = calc_GRF_CoP(q, tau)
        % Calculate ground reaction force (GRF) and CoP location
        % This function assumes a stable foot-hold, i.e. the foot is fixed
        % to the ground plane
        function [ Frx, Fry, CoP ] = calc_GRF_CoP(this, q, tau)
            % Get resampling timestep for differentiation
            Ts_RS = this.params.Ts_RS;

            % Get parameters and calculate total system mass
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            r1      = this.params.r1;
            g       = this.params.g;
            m_tot   = m1 + m2 + m3 + m4;

            % Calculate forward kinematics
            fwdKin  = this.calc_fwdKin(q);
            
            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);

            % Differentiate the forward kinematics twice to obtain the
            % accelerations
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x       = [fwdKin(4,:); fwdKin(7,:); fwdKin(10,:)]; % [x2; x3; x4] because we ignore the foot
            y       = [fwdKin(5,:); fwdKin(8,:); fwdKin(11,:)]; % [y2; y3; y4] because we ignore the foot
            xd      = [[0;0;0] diff(x,1,2)] ./ Ts_RS;
            yd      = [[0;0;0] diff(y,1,2)] ./ Ts_RS;
            xdd     = [[0;0;0] diff(xd,1,2)] ./ Ts_RS;
            ydd     = [[0;0;0] diff(yd,1,2)] ./ Ts_RS;

            % Calculate ground reaction force
            Frx     = m2 * xdd(1,:) + m3 * xdd(2,:) + m4 * xdd(3,:);
            Fry     = m2 * ydd(1,:) + m3 * ydd(2,:) + m4 * ydd(3,:) + m_tot * g;

            % Calculate CoP location
            % The CoP is defined in the global frame using x1
            CoP     = (tau(:,1)' ./ Fry) + x1 - r1;
        end
        
        %__________________________________________________________________
        % Calculates the generalised gravitational force
        function [ G ] = calc_G(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1,:);
            y1      = q(2,:);
            theta1  = q(3,:);
            q1      = q(4,:);
            q2      = q(5,:);
            q3      = q(6,:);

            % Calculate G from the Jacobian

            % Define gravitational force vector
            Gc = [0; -m1*g; 0; 0; -m2*g; 0; 0; -m3*g; 0; 0; -m4*g; 0];
            
            % Calculate G for each configuration in q
            for i=1:size(q,2)
                % Calculate Jacobian
                J = this.calc_J(q(:,i));

                % Calculate G
                G(:,i) = J' * Gc;
            end
            
        end
        
        %__________________________________________________________________
        % Calculates the forward kinematics
        function [ fwdKin ] = calc_fwdKin(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);

            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);

            % Calculate forward kinematics
            fwdKin = [  x1; ...
                        y1; ...
                        theta1; ...
                        x1 - r2 * st1q1 - r1 * ct1; ...
                        y1 + r2 * ct1q1 - r1 * st1; ...
                        theta1 + q1; ...
                        x1 - r3 * st1q12 - l2 * st1q1 - r1 * ct1; ...
                        y1 + r3 * ct1q12 + l2 * ct1q1 - r1 * st1; ...
                        theta1 + q1 + q2; ...
                        x1 - r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1; ...
                        y1 + r4 * ct1q123 + l3 * ct1q12 + l2 * ct1q1 - r1 * st1; ...
                        theta1 + q1 + q3 + q3; ...
            ];
        end
        
        %__________________________________________________________________
        % Calculate the [x,y, theta] location of a named point in the structure
        % Useful to quickly get the hip position for drawing for example.
        % In this case, if the named point is located on a joint (such as the hip),
        % the angle theta is defined as the angle of the previous link.
        function [ x, y, theta ] = calc_fwdKin_named(this, q, name)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1,:);
            y1      = q(2,:);
            theta1  = q(3,:);
            q1      = q(4,:);
            q2      = q(5,:);
            q3      = q(6,:);

            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);

            % Match against the point name
            if (strcmpi(name, 'body') || strcmpi(name, 'trunk')) % Body/trunk
                x       = x1 - r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1;
                y       = y1 + r4 * ct1q123 + l3 * ct1q12 + l2 * ct1q1 - r1 * st1;
                theta   = theta1 + q1 + q3 + q3;
                return;
            
            elseif (strcmpi(name, 'hip'))       % Hip (joint)
                x       = x1 - l3 * st1q12 - l2 * st1q1 - r1 * ct1;
                y       = y1 + l3 * ct1q12 + l2 * ct1q1 - r1 * st1;
                theta   = theta1 + q1 + q2;
                return;
                
            elseif (strcmpi(name, 'knee'))      % Knee
                x       = x1 - l2 * st1q1 - r1 * ct1;
                y       = y1 + l2 * ct1q1 - r1 * st1;
                theta   = theta1 + q1;
                return;
                
            elseif (strcmpi(name, 'ankle'))     % Ankle
                x       = x1 - r1 * ct1;
                y       = y1 - r1 * st1;
                theta   = theta1;
                return;
                
            elseif (strcmpi(name, 'heel'))      % Heel
                x       = x1 - r_heel * ct1;
                y       = y1 - r_heel * st1;
                theta   = theta1;
                return;
                
            elseif (strcmpi(name, 'toe'))       % Toe
                x       = x1 + r_toe * ct1;
                y       = y1 + r_toe * st1;
                theta   = theta1;
                return;
                
            end

            % Show error if the point wasn't found above
            error(['[calc_fwdKin_named] Point "' name '" not found!']);
        end
        
        %__________________________________________________________________
        % Calculate the x-y CoM of the leg
        function [ x, y ] = calc_CoM(this, q)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;
            
            % Get forward kinematics
            fwdKin = this.calc_fwdKin(q);
            
            % Get x-y of each inertia
            x1  = fwdKin(1);
            y1  = fwdKin(2);
            x2  = fwdKin(4);
            y2  = fwdKin(5);
            x3  = fwdKin(7);
            y3  = fwdKin(8);
            x4  = fwdKin(10);
            y4  = fwdKin(11);
            
            % Calculate CoM
            m_tot   = m1 + m2 + m3 + m4;
            x       = (m1 * x1 + m2 * x2 + m3 * x3 + m4 * x4) / m_tot;
            y       = (m1 * y1 + m2 * y2 + m3 * y3 + m4 * y4) / m_tot;
        end
        
        %__________________________________________________________________
        % Calculates the forward velocity kinematics
        function [ fwdKin_vel ] = calc_fwdKin_vel(this, q, q_d)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1);
            y1      = q(2);
            theta1  = q(3);
            q1      = q(4);
            q2      = q(5);
            q3      = q(6);
            
            % Get state derivatives
            x1_d        = q_d(1);
            y1_d        = q_d(2);
            theta1_d    = q_d(3);
            q1_d        = q_d(4);
            q2_d        = q_d(5);
            q3_d        = q_d(6);

            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);

            % Calculate forward velocity kinematics
            fwdKin_vel = [  x1_d; ...
                            y1_d; ...
                            theta1_d; ...
                            x1_d + theta1_d * (r1 * st1 - r2 * ct1q1) - q1_d * r2 * ct1q1; ...
                            y1_d + theta1_d * (-r2 * st1q1 - r1 * ct1) - q1_d * r2 * st1q1; ...
                            theta1_d + q1_d; ...
                            x1_d + theta1_d * (-r3 * ct1q12 - l2 * ct1q1 + r1 * st1) + q1_d * (-r3 * ct1q12 - l2 * ct1q1) - q2_d * r3 * ct1q12; ...
                            y1_d + theta1_d * (-r3 * st1q12 - l2 * st1q1 - r1 * ct1) + q1_d * (-r3 * st1q12 - l3 * st1q1) - q2_2 * r3 * st1q12; ...
                            theta1_d + q1_d + q2_d; ...
                            x1_d + theta1_d * (-r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1 + r1 * st1) + q1_d * (-r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1) + q2_d * (-r4 * ct1q123 - l3 * ct1q12) - q3_d * r4 * ct1q123; ...
                            y1_d + theta1_d * (-r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1) + q1_d * (-r4 * st1q123 - l3 * st1q12 - l2 * st1q1) + q2_d * (-r4 * st1q123 - l3 * st1q12) - q3_d * r4 * st1q123; ...
                            theta1_d + q1_d + q2_d + q3_d; ...
            ];
        end
        
        %__________________________________________________________________
        % Calculate the Cartesian velocity of a named point in the structure
        % If the named point is located on a joint (such as the hip),
        % the angle theta is defined as the angle of the previous link.
        function [ x_d, y_d, theta_d ] = calc_fwdKin_vel_named(this, q, q_d, name)
            % Get parameters
            m1      = this.params.m1;
            m2      = this.params.m2;
            m3      = this.params.m3;
            m4      = this.params.m4;
            J1      = this.params.J1;
            J2      = this.params.J2;
            J3      = this.params.J3;
            J4      = this.params.J4;
            l1      = this.params.l1;
            l2      = this.params.l2;
            l3      = this.params.l3;
            l4      = this.params.l4;
            r_toe   = this.params.r_toe;
            r_heel  = this.params.r_heel;
            r1      = this.params.r1;
            r2      = this.params.r2;
            r3      = this.params.r3;
            r4      = this.params.r4;
            g       = this.params.g;

            % Get states
            % q = [x1, y1, theta1, q1, q2, q3]
            % x = [x1, y1, theta1, x2, y2, theta2, x3, y3, theta3, x4, y4, theta4]
            x1      = q(1,:);
            y1      = q(2,:);
            theta1  = q(3,:);
            q1      = q(4,:);
            q2      = q(5,:);
            q3      = q(6,:);
            
            % Get state derivatives
            x1_d        = q_d(1);
            y1_d        = q_d(2);
            theta1_d    = q_d(3);
            q1_d        = q_d(4);
            q2_d        = q_d(5);
            q3_d        = q_d(6);

            % Calculate some (co)sines
            ct1         = cos(theta1);
            st1         = sin(theta1);
            ct1q1       = cos(theta1 + q1);
            st1q1       = sin(theta1 + q1);
            ct1q12      = cos(theta1 + q1 + q2);
            st1q12      = sin(theta1 + q1 + q2);
            ct1q123     = cos(theta1 + q1 + q2 + q3);
            st1q123     = sin(theta1 + q1 + q2 + q3);

            % Match against the point name
            if (strcmpi(name, 'body') || strcmpi(name, 'trunk')) % Body/trunk
                x_d         = x1_d + theta1_d * (-r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1 + r1 * st1) + q1_d * (-r4 * ct1q123 - l3 * ct1q12 - l2 * ct1q1) + q2_d * (-r4 * ct1q123 - l3 * ct1q12) - q3_d * r4 * ct1q123;
                y_d         = y1_d + theta1_d * (-r4 * st1q123 - l3 * st1q12 - l2 * st1q1 - r1 * ct1) + q1_d * (-r4 * st1q123 - l3 * st1q12 - l2 * st1q1) + q2_d * (-r4 * st1q123 - l3 * st1q12) - q3_d * r4 * st1q123;
                theta_d     = theta1_d + q1_d + q2_d + q3_d;
                return;
            
            elseif (strcmpi(name, 'hip'))       % Hip (joint)
                x_d         = x1_d + theta1_d * (-l3 * ct1q12 - l2 * ct1q1 + r1 * st1) + q1_d * (-l3 * ct1q12 - l2 * ct1q1) - q2_d * l3 * ct1q12;
                y_d         = y1_d + theta1_d * (-l3 * st1q12 - l2 * st1q1 - r1 * ct1) + q1_d * (-l3 * st1q12 - l3 * st1q1) - q2_2 * l3 * st1q12;
                theta_d     = theta1_d + q1_d + q2_d;
                return;
                
            elseif (strcmpi(name, 'knee'))      % Knee
                x_d         = x1_d + theta1_d * (r1 * st1 - l2 * ct1q1) - q1_d * l2 * ct1q1;
                y_d         = y1_d + theta1_d * (-l2 * st1q1 - r1 * ct1) - q1_d * l2 * st1q1;
                theta_d     = theta1_d + q1_d;
                return;
                
            elseif (strcmpi(name, 'ankle'))     % Ankle
                x_d         = x1_d + theta1_d * r1 * st1;
                y_d         = y1_d - theta1_d * r1 * ct1;
                theta_d     = theta1_d;
                return;
                
            elseif (strcmpi(name, 'heel'))      % Heel
                x_d         = x1_d + theta1_d * r_heel * st1;
                y_d         = y1_d - theta1_d * r_heel * ct1;
                theta_d     = theta1_d;
                return;
                
            elseif (strcmpi(name, 'toe'))       % Toe
                x_d         = x1_d - theta1_d * r_toe * st1;
                y_d         = y1_d + theta1_d * r_toe * ct1;
                theta_d     = theta1_d;
                return;
                
            end

            % Show error if the point wasn't found above
            error(['[calc_fwdKin_named] Point "' name '" not found!']);
        end

    end
    
end
