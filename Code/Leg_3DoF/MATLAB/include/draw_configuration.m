function [ ] = draw_configuration(leg, q, tau, t, infoText)
% [ ] = draw_configuration(leg, q, tau, t, infoText)
% Draw the leg in a given configuration and show the static (fake) CoP:
% draw_configuration(leg, q);
% To also show the proper CoP, make sure q is a time series and supply tau:
% draw_configuration(leg, q, tau);
% Adding t and infoText arguments simply shows them as text

    % Tau parameter is optional
    if (~exist('tau', 'var'))
        tau = -1;
    end
    % Time parameter is optional
    if (~exist('t', 'var'))
        t = -1;
    end

    % Clear current axes
    cla; hold on;

    % Get the positions of a few points for drawing
    [x_toe, y_toe, ~]       = leg.calc_fwdKin_named(q(:,end), 'toe');
    [x_heel, y_heel, ~]     = leg.calc_fwdKin_named(q(:,end), 'heel');
    [x_ankle, y_ankle, ~]	= leg.calc_fwdKin_named(q(:,end), 'ankle');
    [x_knee, y_knee, ~]     = leg.calc_fwdKin_named(q(:,end), 'knee');
    [x_hip, y_hip, ~]       = leg.calc_fwdKin_named(q(:,end), 'hip');
    [x_body, y_body, ~]     = leg.calc_fwdKin_named(q(:,end), 'body');
    
    % Get the CoM position
    [x_CoM, y_CoM]          = leg.calc_CoM(q(:,end));

    % Floor, dotted helper line
    plot([-0.2 0.6], [-0.01 -0.01], 'black', 'LineWidth', 2);
%     plot([0 0], [0 1.1], '--', 'Color', 'black');

    % Leg segments
    plot([x_heel, x_toe], [y_heel, y_toe], 'b', 'LineWidth', 3);
    plot([x_ankle, x_knee], [y_ankle, y_knee], 'b', 'LineWidth', 3);
    plot([x_knee, x_hip], [y_knee, y_hip], 'b', 'LineWidth', 3);
    plot([x_hip, x_body], [y_hip, y_body], 'b', 'LineWidth', 3);
    
    % CoM
    scatter(x_CoM, y_CoM, 25,'MarkerFaceColor','r','MarkerEdgeColor','r');
    
    % CoP
    if (size(q,1) == size(tau,2))
        % Calculate and plot static CoP
        [~, CoP] = leg.calc_GRF_CoP_static(q(:,end));
        plot([CoP, CoP], [-0.02, 0.1], 'r', 'LineWidth', 3);
        
        % Calculate and plot actual CoP
        [~, ~, CoP] = leg.calc_GRF_CoP(q, tau(1:size(q,2),:));
        plot([CoP(end), CoP(end)], [-0.02, 0.1], 'g', 'LineWidth', 3);
    else
        % Calculate and plot static CoP
        [~, CoP] = leg.calc_GRF_CoP_static(q(:,end));
        plot([CoP, CoP], [-0.02, 0.1], 'r', 'LineWidth', 3);
    end

    % Plot settings
    %xlim([-0.35 0.68]);
    %ylim([-0.1 1.2]);
    xlim([-0.5 0.9]);
    ylim([-0.1 1.3]);
     axis square
    
    % Plot time if available
    if (t > 0)
        text(-0.4, 1.2, ['t = ' num2str(t, '%2.2f') ' s'], 'FontSize', 16);
    end
    
    % Show info text if available
    if (exist('infoText', 'var'))
        text(0.2, 0.6, infoText, 'FontSize', 16, 'Color','red');
    end

end

