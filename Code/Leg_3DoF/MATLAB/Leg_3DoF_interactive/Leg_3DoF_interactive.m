% Leg_3DoF_interactive.m
% Present an interactive GUI to the user where the leg can be manipulated
% and additional information is shown.

function [] = Leg_3DoF_interactive(legParamsFileName)
    %% Default parameters
    if (~exist('legParamsFileName', 'var'))
        legParamsFileName = '';
    end

    %% Build leg object
    leg = Leg_3DoF(legParamsFileName);


    %% Initial configuration
    d2r = (pi/180);
    q1 = -20 * d2r;
    q2 = 40 * d2r;
    q3 = -20 * d2r;
    q = [0;0;0; q1; q2; q3];

    % Joint limits
    q_limits = leg.params.q_limits;


    %% Calculate forward kinematics, gravity, inertia
    %fwdKin	= calc_fwdKin([0;0;0; q1';q2';q3'], params);
    %G       = calc_G(q, params);
    %M       = calc_M(q, params);


%      %% Get figure
%     hFig = figure; clf;
%     pos = get(hFig, 'Position');
%     xPos = pos(1); yPos = pos(2);
%     set(hFig, 'Position', [xPos yPos 800 300]);%800 500]);

    % Open in left half of screen
    hFig = figure; clf;
    set(hFig, 'Position', [100 300 800 600]);

    %% Add GUI

    % Plotting axes
    h_axes = axes('position', [0.08 0.15 0.4 0.8]);
    hold on; grid on;

    % q1 slider
    h_q1_slider = uicontrol(...
                'Style', 'slider', ...
                'Min', q_limits(1,1), 'Max', q_limits(1,2), 'Value', q1, ...
                'Position', [0 30 150 20], ...
                'Callback', {@handle_gui, hFig}, ...
                'UserData', struct('name', 'q1_slider')     );

    % q2 slider
    h_q2_slider = uicontrol(...
                'Style', 'slider', ...
                'Min', q_limits(2,1), 'Max', q_limits(2,2), 'Value', q2, ...
                'Position', [150 30 150 20], ...
                'Callback', {@handle_gui, hFig}, ...
                'UserData', struct('name', 'q2_slider')     );

    % q3 slider
    h_q3_slider = uicontrol(...
                'Style', 'slider', ...
                'Min', q_limits(3,1), 'Max', q_limits(3,2), 'Value', q3, ...
                'Position', [300 30 150 20], ...
                'Callback', {@handle_gui, hFig}, ...
                'UserData', struct('name', 'q3_slider')     );

    % q1 value text
    h_q1_text = uicontrol(...  
                'Style', 'text', ...
                'Position', [0 15 150 15], ...
                'String', ['q1: ' num2str(q1, 3) ' [rad]']  );

    % q2 value text
    h_q2_text = uicontrol(...  
                'Style', 'text', ...
                'Position', [150 15 150 15], ...
                'String', ['q2: ' num2str(q2, 3) ' [rad]']  );

    % q3 value text
    h_q3_text = uicontrol(...  
                'Style', 'text', ...
                'Position', [300 15 150 15], ...
                'String', ['q3: ' num2str(q3, 3) ' [rad]']  );

    % Info text
    h_info_text = uicontrol(...  
                'Style', 'text', ...
                'Position', [430 75 350 400], ...
                'String', info_text(leg, q));
            
    % q3 lock checkbox
    h_q3_lock = uicontrol(...
                'Style', 'checkbox', ...
                'String', 'Lock trunk vertically', ...
                'Position', [480 30 150 20], ...
                'Callback', {@handle_gui, hFig}, ...
                'UserData', struct('name', 'q3_lock')     );


    %% Set figure's UserData field to store useful stuff
    UserData.leg            = leg;              % Leg object
    UserData.q              = q;                % Current configuration
    UserData.h_axes         = h_axes;           % Plot axes handle
    UserData.h_q1_slider    = h_q1_slider;      % q1 slider handle
    UserData.h_q2_slider    = h_q2_slider;      % q2 slider handle
    UserData.h_q3_slider    = h_q3_slider;      % q3 slider handle
    UserData.h_q3_lock      = h_q3_lock;        % q3 lock checkbox handle
    UserData.h_q1_text      = h_q1_text;        % q1 text handle
    UserData.h_q2_text      = h_q2_text;        % q2 text handle
    UserData.h_q3_text      = h_q3_text;        % q3 text handle
    UserData.h_info_text   	= h_info_text;    	% Info text handle

    % Set field into figure
    set(hFig, 'UserData', UserData);


    %% Perform drawing of leg
    draw_configuration(leg, q);
    

    %% Finish up
    disp('Finished.');
    
end

