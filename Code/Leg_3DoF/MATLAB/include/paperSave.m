function [ ] = paperSave( paperMode, filename )
%paperSave Save figure to file if paperMode is ON

    if (paperMode)
        export_fig(filename, '-transparent');
        disp(['[paperSave] Saved figure to file ''' filename '''']);
    end

end

