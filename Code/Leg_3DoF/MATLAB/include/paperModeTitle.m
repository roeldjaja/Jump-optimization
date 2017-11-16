function [ ] = paperModeTitle( showTitle, paperMode, title_paperMode, title_normal )
    % Don't do anything if showTitle==0 and we are in paper mode
    if (~showTitle && paperMode)
        return;
    end

    % Default arguments
    if (~exist('title_normal', 'var') || isequal(title_normal,[]))
        title_normal = title_paperMode;
    end
    
    % Place title depending on paperMode
    if (paperMode)
        title(title_paperMode, 'Interpreter', 'LaTeX');
    else
        title(title_normal);
    end
end