% [ params ] = ACA_test_params()
% Parameter file for the ACA package
function [ params ] = ACA_test_params()

    % Define constants and parameters

    % Name for this set of parameters
    params.name     = 'ACA_test_params';

    % Constants
    params.g        = 9.81;                     % Gravitational constant [m s^-2]
    
    % Misc
    params.fps      = 60;                       % Framerate for resampling
    params.Ts_RS    = 1/params.fps;             % Timestep for resampling

    % Power Branch = PB
    % Energy Storage Branch = ESB
    
    % PB motor
    % Kollmorgen TBM(S)-6025-B (24V version running at 48V)
    % CPL-20A-80-?-?
    params.d_e1     = 0.318;                    % PB motor electrical resistance [Ohm]
    params.H_1      = 0.20e-3;                  % PB motor inductance [H]
    params.k_t1     = 0.085;                    % PB motor torque constant [N m / A]
    params.I_m1     = 2.52e-5 + 0.0 + 0.263e-4; % PB + HD inertia [kg m^2] (CHECK HD AND MOUNTING INERTIA)
    params.d_m1     = 2.0e-3 + 2.0e-3;          % PB + HD damping [N m s / rad] (CHECK HD DAMPING) (PB: 5.35e-3 Nm/kRPM = 2e-3 Nms/rad)
    params.r_m1     = 80;                       % Harmonic drive transmission ratio []
    params.v_1_max  = 80;                       % PB motor max voltage [V] %48
    params.i_1_max  = 30;                       % PB motor max current [A]

    % ESB motor
    % Maxon EC22 100W 48V: Part #386675
    % Maxon GP22 HP gearbox 53:1: Part #370776
    params.d_e2     = 0.797;                    % ESB motor electrical resistance [Ohm]
    params.H_2      = 0.118e-3;                 % ESB motor inductance [H]
    params.k_t2     = 14.2e-3;                  % ESB motor torque constant [N m / A]
    params.I_m2     = 4.09e-7 + 0.4e-7;         % ESB motor + gearbox inertia [kg m^2]
    params.d_m2     = 0.2e-5; %guess            % ESB motor damping [N m s / rad]
    params.r_m2     = 53 * (2*pi) / 0.005;      % ESB gearbox + ball screw transmission ratio [rad/m]
    % for 29:1 motors: 29.16 * (2*pi) / 0.005;
    % for old 1-DoF prototype: 29.16 * (2*pi) / 0.01;
    params.v_2_max  = 48;                       % ESB motor max voltage [V]
    params.i_2_max  = 4;                        % ESB motor max current [A]
    
    % Pretension range
    % Default value, will be overwritten by leg actuation parameters
    params.p_range  = [-0.08 0.08];             % Pretension range [[m] [m]]

    % PB spring
    params.k_pb     = 6500;                     % PB spring stiffness [N m]
    params.d_pb     = 0.01;                     % PB spring damping [N m s / rad]

    % ESB spring
    % Default value, will be overwritten by leg actuation parameters
    params.k_p      = 8.7e3;                    % ESB spring(s) stiffness [N / m]
    params.d_p      = 0.2;                      % ESB spring(s) damping [N s / m]
    
end
