function [ q ] = handle_gui( hObj, event, hFig ) %#ok<*INUSL>
    %#ok<*NASGU>
    
    % Get UserData
    UserData	= get(hFig, 'UserData');
    leg         = UserData.leg;             % Leg object
    q           = UserData.q;               % Current configuration
    h_axes      = UserData.h_axes;          % Plot axes handle
    h_q1_slider = UserData.h_q1_slider;     % q1 slider handle 
    h_q2_slider = UserData.h_q2_slider;     % q2 slider handle
    h_q3_slider = UserData.h_q3_slider;     % q3 slider handle
    h_q1_text 	= UserData.h_q1_text;       % q1 text handle
    h_q2_text  	= UserData.h_q2_text;     	% q2 text handle
    h_q3_text  	= UserData.h_q3_text;      	% q3 text handle
    h_q3_lock   = UserData.h_q3_lock;       % q3 lock checkbox handle
    h_info_text = UserData.h_info_text;     % Info text handle

    % Get control UserData and value
    hObj_UserData   = get(hObj, 'UserData');
    val             = get(hObj, 'value');
    
    % Set the right value depending on the name and update text label
    if (strcmpi(hObj_UserData.name, 'q1_slider'))
        q(4) = val;
        set(h_q1_text, 'String', ['q1: ' num2str(q(1), 3) ' [rad]']);
    elseif (strcmpi(hObj_UserData.name, 'q2_slider'))
        q(5) = val;
        set(h_q2_text, 'String', ['q2: ' num2str(q(2), 3) ' [rad]']);
    elseif (strcmpi(hObj_UserData.name, 'q3_slider'))
        q(6) = val;
        set(h_q3_text, 'String', ['q3: ' num2str(q(3), 3) ' [rad]']);
    elseif (strcmpi(hObj_UserData.name, 'q3_lock'))
        % Don't have to do anything here (see below)
    else
        error('[handle_gui] Error: Unknown control!');
    end
    
    % Lock the trunk in place if the checkbox is set
    if (h_q3_lock.Value == 1)
        q(6) = -(q(4) + q(5));
        h_q3_slider.Value = q(6);
        set(h_q3_text, 'String', ['q3: ' num2str(q(6), 3) ' [rad]']);
    end
    
    % Set back into UserData
    UserData.q  = q;
    set(hFig, 'UserData', UserData);
	
    % Re-draw configuration
    draw_configuration(leg, q);
    
    % Update info text
    set(h_info_text, 'String', info_text(leg, q));
    
end

