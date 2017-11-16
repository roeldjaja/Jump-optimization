function [ output ] = info_text(leg, q)
% [ ] = info_text( q, params )
% Update the info text on the side of the interactive plot

    % Calculate CoP
    [Fry, CoP] = leg.calc_GRF_CoP_static(q);

    % Calculate generalised gravitation vector
    G = leg.calc_G(q);
    
    % Calculate inertia matrix
    M = leg.calc_M(q);
    
    % Calculate polar coordinates of hip
    [x_hip, y_hip] = leg.calc_fwdKin_named(q, 'hip');
    beta	= atan2(-x_hip, y_hip);
    r     	= sqrt(x_hip^2 + y_hip^2);

    % Create output string
    output = {  ['Fry: ' num2str(Fry) ' N'], '', ...
                ['CoP: ' num2str(CoP) ' m'], '', ...
                'G`:', num2str(G', 3), '', ...
                'M:', num2str(M, 3), '', ...
                'Polar coordinates of hip:', ...
                ['[beta, r] =  [' num2str(beta, '%4.2f') ',' num2str(r, '%4.2f') ']'],'', ...
                'Configuration q in rad: ', num2str(q',3) ''};

end

