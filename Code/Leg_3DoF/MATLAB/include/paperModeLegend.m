function [ h ] = paperModeLegend( paperMode, data_paperMode, data_normal, plotHandles )
    % Default arguments
    if (~exist('data_normal', 'var'))
        data_normal = data_paperMode;
    end
    
    % Set right data
    if (paperMode || isempty(data_normal))
        data = data_paperMode;
    else
        data = data_normal;
    end

    % Place legend
    if (exist('plotHandles', 'var') && length(plotHandles)==length(data))
        h = legend(plotHandles, data, 'Location', 'best');
    else
        h = legend(data, 'Location', 'best');
    end
    
    % Set interpreter in paper mode
    if (paperMode)
        set(h, 'Interpreter', 'LaTeX');
    end
end