function [ ] = setYAxis( s, ignoreXRange )
%SETYAXIS Set the plot y-axis in a nice way: Data range + percentage.
%[ ] = setYAxis( scale, ignoreXRange )
%of the range on both sides. s is specified as a fraction (default: 0.03).
%The ignoreXRange parameter allows to ignore a specified first part of the
%x-range, to eliminate start-up behaviour from setting the y-axis.

    % Check if s exists. If not, set default value. -1 is default.
    if ~exist('s','var')
        s = 0.03;
    end
    if (s < 0)
        s = 0.03;
    end
    
    % Get initial axis value to make sure x doesn't change
    a0 = axis;
    
    % When ignoring part of the x-axis, we still show that x-range but we
    % simply ignore it for setting the y-axis
    if (exist('ignoreXRange', 'var'))
        if (~isnumeric(ignoreXRange) || length(ignoreXRange) ~= 1)
            error('[setYAxis] ignoreXRange is not a number');
        end
        
        % Set axis tight
        axis tight;
        
        % Set x-range ignoring the ignoreXRange part, and set y-range to
        % auto to obtain tight fit there
        xlim([ignoreXRange a0(2)]);
        ylim('auto');
        
        % Obtain current axis values
        a1 = axis;
        
        % Get y range
        r = range([a1(3), a1(4)]);
        
        % Set new axis values
        axis([a0(1) a0(2) a1(3) a1(4)] + [0 0 -s*r s*r]);
        
    else
        % Set axis tight
        axis tight;
        
        % Get current axis value
        a1 = axis;
        
        % Re-set old x-axis, and set y-range to auto to obtain tight fit
        % there
        axis([a0(1) a0(2) a1(3) a1(4)]);
        ylim('auto');

        % Get current axis value
        a1 = axis;

        % Get y range
        r = range([a1(3), a1(4)]);

        % Set new axis values
        axis([a0(1) a0(2) a1(3) a1(4)] + [0 0 -s*r s*r]);
        
    end

end

