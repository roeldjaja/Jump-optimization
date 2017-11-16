% [ params ] = Leg_3DoF_design_noESB()
% Parameter file for the Leg_3DoF package - SEA-only (noESB) configuration
function [ params ] = Leg_3DoF_design_noESB()

    % Define constants and parameters

    % Name for this set of parameters
    params.name     = 'Leg_3DoF_design_noESB';

    % Constants
    params.g        = 9.81;             % Gravitational constant [m s^-2]

    % Dimensions
    params.l1       = 0.30;             % Link 1 length [m]
    params.l2       = 0.35;             % Link 2 length [m]
    params.l3       = 0.35;             % Link 3 length [m]
    params.l4       = 0.30;             % Link 4 length [m]
    params.r_toe    = 0.186;            % Foot CoM to toe [m]
    params.r_heel   = params.l1 - params.r_toe; % Foot CoM to heel [m]
    
    % Joint locations
    params.r1       = 0.01105;          % Joint location on link 1 [m]
    params.r2       = 0.21472;          % Joint location on link 2 [m]
    params.r3       = 0.21479;          % Joint location on link 3 [m]
    params.r4       = params.l4;    	% Joint location on link 4 [m]

    % Masses/inertiae
    params.m1       = 1.703;            % Link 1 mass [kg]
    params.J1       = 0.0081;           % Link 1 rot. inertia [kg m^2]
    %params.m1 * params.l1^2 / 12; (cilinder around midpoint)
    
    params.m2       = 3.080;            % Link 2 mass [kg] (link + motors)
    params.J2       = 0.0226;           % Link 2 rot. inertia [kg m^2]

    params.m3       = 2.791;            % Link 3 mass [kg] (link + motors)
    params.J3       = 0.0246;           % Link 3 rot. inertia [kg m^2]

    params.m4       = 20;               % Link 4 mass [kg]
    params.J4       = (2/5) * params.m4 * params.l4^2; % Link 4 rot. inertia [kg m^2] (solid sphere, r = l4)

    % Joint friction
    params.d        = [0.0; 0.0; 0.0; 0.1; 0.1; 0.1];  % Damping on states [Ns/m] and joints [Nms/rad]
    
    % Ground contact stiffness & friction coefficients
    % http://www.engineeringtoolbox.com/young-modulus-d_417.html
    % http://www.engineeringtoolbox.com/friction-coefficients-d_778.html
    % Stiffness: 0.1e9 Pa or N/m^2 (rubber)
    % Surface area: 0.26 * 0.15 = 0.039 m^2 (approx. foot surface area)
    % Thickness (length): 0.004 m.
    % Effective stiffness: 0.039 * 0.1e9 / 0.004  = 975e6 N/m;
    % This is too stiff to simulate
    params.floor_K  = 100000;           % Floor y-stiffness [N/m]
    params.floor_D  = 500;              % Floor y-damping [Ns/m] (guess)
    params.mu       = 0.5;              % Floor viscous friction coefficient (rubber-asphalt) [Ns/m]
    params.coulomb  = 50;               % Floor Coulomb friction coefficient [N] (guess)
    
    % Joint limits [lb, ub]
    d2r = pi/180; % Degrees to radians
    params.q_limits = [ -64 * d2r       53 * d2r; ...       % q_1
                        -1.3 * d2r      130 * d2r; ...      % q_2
                        -90 * d2r       90 * d2r   ];       % q_3

    % Pretension position ranges
    % Monoarticulated
    params.p_range_mono = [ -0.06       0.06; ...   % p_1 (ESB motor on ankle)
                            -0.03       0.074; ...  % p_2 (ESB motor on top of thigh)
                            0           0       ];  % p_3 (none)
    % Biarticulated
    params.p_range_bi   = [ -0.024      0.044; ...  % p_1 (ESB motor on bottom of thigh)
                            -0.03       0.074; ...  % p_2 (ESB motor on top of thigh)
                            0           0       ];  % p_3 (none)
    
    % Misc
    params.fps      = 60;               % Framerate for resampling
    params.Ts_RS    = 1/params.fps;     % Timestep for resampling
    
end
