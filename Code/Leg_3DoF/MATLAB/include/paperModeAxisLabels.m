function [ ] = paperModeAxisLabels( paperMode, labels_paperMode, labels_normal )
    % Default arguments
    if (~exist('labels_normal', 'var') || isequal(labels_normal,[]))
        labels_normal = labels_paperMode;
    end
    
    % Place labels depending on paperMode
    if (paperMode)
        xlabel(labels_paperMode{1}, 'Interpreter', 'LaTeX');
        ylabel(labels_paperMode{2}, 'Interpreter', 'LaTeX');
    else
        xlabel(labels_normal{1});
        ylabel(labels_normal{2});
    end
end