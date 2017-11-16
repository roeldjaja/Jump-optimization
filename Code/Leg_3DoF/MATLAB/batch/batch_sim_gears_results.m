% batch_sim_gears_results.m
% Process and optionally plot the result of the batch simulations

function [] = batch_sim_gears_results(filename)
    % Default parameters
    if (~exist('filename', 'var') || isempty(filename))
        error('No filename given!');
    end

    % Load data
    % Check if file exists first
    if (~exist(filename, 'file'))
        error(['File ''' filename ''' does not exist!']);
    end
    load(filename);

    
    %% Go through all simulators
    for i=1:size(sim_data,1)
        for j=1:size(sim_data,2)
            % Get simulator, actuator parameters filename and gearing ratio
            sim                 = sim_data(i,j);
            actParamsFileName	= sim.model.actParamsFileName;
            gear_ratio          = gear_ratios(j);

            % Time
            t           = sim.data.t;

            % Get motor powers
            P_A11       = sim.data.y_a1(sim.model.a1.outputIdx_P_1,:);
            P_A12       = sim.data.y_a1(sim.model.a1.outputIdx_P_2,:);
            P_A21       = sim.data.y_a2(sim.model.a2.outputIdx_P_1,:);
            P_A22       = sim.data.y_a2(sim.model.a2.outputIdx_P_2,:);
            P_A31       = sim.data.y_a3(sim.model.a3.outputIdx_P_1,:);
            P_A32       = sim.data.y_a3(sim.model.a3.outputIdx_P_2,:);
            
            % Restrict to positive (delivered) power in _pos vars
            P_A11_pos = P_A11;
            P_A12_pos = P_A12;
            P_A21_pos = P_A21;
            P_A22_pos = P_A22;
            P_A31_pos = P_A31;
            P_A32_pos = P_A32;
            P_A11_pos(P_A11_pos<0) = 0;
            P_A12_pos(P_A12_pos<0) = 0;
            P_A21_pos(P_A21_pos<0) = 0;
            P_A22_pos(P_A22_pos<0) = 0;
            P_A31_pos(P_A31_pos<0) = 0;
            P_A32_pos(P_A32_pos<0) = 0;
            
            % Sum motors to get actuator power and actuators to get system power
            P_A1        = P_A11_pos + P_A12_pos;
            P_A2        = P_A21_pos + P_A22_pos;
            P_A3        = P_A31_pos + P_A32_pos;
            P           = P_A1 + P_A2 + P_A3;

            % Filter powers
            windowSize  = sim.model.ref.T / sim.params.Ts;
            P_A11_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A11);
            P_A12_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A12);
            P_A21_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A21);
            P_A22_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A22);
            P_A31_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A31);
            P_A32_filt  = filter(ones(1,windowSize) / windowSize, 1, P_A32);
            P_A1_filt   = filter(ones(1,windowSize) / windowSize, 1, P_A1);
            P_A2_filt   = filter(ones(1,windowSize) / windowSize, 1, P_A2);
            P_A3_filt   = filter(ones(1,windowSize) / windowSize, 1, P_A3);
            P_filt      = filter(ones(1,windowSize) / windowSize, 1, P);
            
            % Pulley radii
            t_1         = sim.model.a1.topology.t;
            t_2         = sim.model.a2.topology.t;
            radii_1     = num2str(abs(nonzeros(t_1)'));
            radii_2     = num2str(abs(nonzeros(t_2)'));

            % Maximum ESB extension
            dL_p_1_max  = max(sim.data.x_a1(sim.model.a1.stateIdx_dL_p,:));
            dL_p_2_max  = max(sim.data.x_a2(sim.model.a2.stateIdx_dL_p,:));
            dL_p_3_max  = max(sim.data.x_a3(sim.model.a3.stateIdx_dL_p,:));

            % Maximum tensile ESB forces
            F_p_1_max	= dL_p_1_max * sim.model.a1.params.k_p;
            F_p_2_max	= dL_p_2_max * sim.model.a2.params.k_p;
            F_p_3_max	= dL_p_3_max * sim.model.a3.params.k_p;

            % Minimum/maximum pretension positions
            p_1_min     = min(sim.data.x_a1(sim.model.a1.stateIdx_p,:));
            p_2_min     = min(sim.data.x_a2(sim.model.a2.stateIdx_p,:));
            p_3_min     = min(sim.data.x_a3(sim.model.a3.stateIdx_p,:));
            p_1_max     = max(sim.data.x_a1(sim.model.a1.stateIdx_p,:));
            p_2_max     = max(sim.data.x_a2(sim.model.a2.stateIdx_p,:));
            p_3_max     = max(sim.data.x_a3(sim.model.a3.stateIdx_p,:));

            % Create and add table row
            row.name                = ['sim ' num2str((i-1)*size(sim_data,2) + j)];
            row.actParamsFileName	= actParamsFileName;
            row.gear_ratio          = gear_ratio;
            row.P_A1                = P_A1_filt(end);
            row.P_A2                = P_A2_filt(end);
            row.P_A3                = P_A3_filt(end);
            row.P                   = P_filt(end);
            row.k_p_1               = sim.model.a1.params.k_p;
            row.k_p_2               = sim.model.a2.params.k_p;
            % No ESB on actuator 3
            row.radii_1             = radii_1;
            row.radii_2             = radii_2;
            row.dL_p_1_max          = dL_p_1_max;
            row.dL_p_2_max          = dL_p_2_max;
            row.dL_p_3_max          = dL_p_3_max;
            row.F_p_1_max           = F_p_1_max;
            row.F_p_2_max           = F_p_2_max;
            row.F_p_3_max           = F_p_3_max;
            row.p_1_min             = p_1_min;
            row.p_2_min             = p_2_min;
            row.p_3_min             = p_3_min;
            row.p_1_max             = p_1_max;
            row.p_2_max             = p_2_max;
            row.p_3_max             = p_3_max;
            if (exist('tableData', 'var'))
                tableData(end+1)        = row; %#ok<*SAGROW>
            else
                tableData               = row;
            end

            % Add data to plotting data store
            clear plotItem
            idxs                        = round(length(t)/2):length(t);
            plotItem.actParamsFileName	= actParamsFileName;
            plotItem.gear_ratio         = gear_ratio;
            plotItem.q_rotor_dot_1    	= max(abs(sim.data.x_a1(sim.model.a1.stateIdx_p_d,idxs))) * sim.model.a1.params.r_m2;
            plotItem.q_rotor_dot_2     	= max(abs(sim.data.x_a2(sim.model.a2.stateIdx_p_d,idxs))) * sim.model.a2.params.r_m2;
            plotItem.P_A11              = P_A11_filt(end);
            plotItem.P_A12              = P_A12_filt(end);
            plotItem.P_A21              = P_A21_filt(end);
            plotItem.P_A22              = P_A22_filt(end);
            plotItem.P_A31              = P_A31_filt(end);
            plotItem.P_A32              = P_A32_filt(end);
            plotItem.P_A1               = P_A1_filt(end);
            plotItem.P_A2               = P_A2_filt(end);
            plotItem.P_A3               = P_A3_filt(end);

            if (exist('plot_data', 'var'))
                plot_data(end+1)        = plotItem;
            else
                plot_data               = plotItem;
            end

            % To skip plots, continue here
            continue;

            % M2 rotor velocities
            figure; hold on; grid on;
            plot(t, sim.data.x_a1(sim.model.a1.stateIdx_p_d,:) * sim.model.a1.params.r_m2);
            plot(t, sim.data.x_a2(sim.model.a2.stateIdx_p_d,:) * sim.model.a2.params.r_m2);
            plot(t, sim.data.x_a3(sim.model.a3.stateIdx_p_d,:) * sim.model.a3.params.r_m2);
            legend('p_{d,1}', 'p_{d,2}', 'p_{d,3}');
            xlabel('t [s]'); ylabel('[rad/s]');
            setYAxis;
            title(['M2 rotor velocities for (i,j)=(' num2str(i) ',' num2str(j) ')']);

            % To skip plots, continue here
            continue;

            % Plot
            figure; hold on; grid on;
            plot(t, P_A1, 'Color', [0.6 0.6 1.0]);
            plot(t, P_A2, 'Color', [1.0 0.6 0.6]);
            plot(t, P_A3, 'Color', [0.6 1.0 0.6]);
            plot(t, P, 'Color', [0.4 0.4 0.4]);
            plot(t, P_A1_filt, 'Color', [0.3 0.3 1.0], 'LineWidth', 2);
            plot(t, P_A2_filt, 'Color', [1.0 0.4 0.4], 'LineWidth', 2);
            plot(t, P_A3_filt, 'Color', [0.3 1.0 0.3], 'LineWidth', 2);
            plot(t, P_filt, 'Color', 'black', 'LineWidth', 2);
            setYAxis;
            legend( 'P_{A1}', 'P_{A2}', 'P_{A3}', 'P', ...
                    ['P_{A1} filt. (' num2str(P_A1_filt(end), '%3.1f') ' W end)'], ...
                    ['P_{A2} filt. (' num2str(P_A2_filt(end), '%3.1f') ' W end)'], ...
                    ['P_{A3} filt. (' num2str(P_A3_filt(end), '%3.1f') ' W end)'], ...
                    ['P filt. (' num2str(P_filt(end), '%3.1f') ' W end)'], ...
                    'Location', 'NorthEast');
            xlabel('t [s]'); ylabel('[W]');
            title(['Electrical power for params ''' actParamsFileName ''' and total trunk weight ' num2str(sim.model.leg.params.m3) ' kg']);
        end
    end


    %% Show table
    tabl1 = table(  {tableData.actParamsFileName}', [tableData.gear_ratio]', ...
                    [tableData.P_A1]', [tableData.P_A2]', [tableData.P_A3]', [tableData.P]', ...
                    'RowNames', {tableData.name}', ...
                    'VariableNames', {'actParamsFileName', 'gear_ratio', 'P_A1', 'P_A2', 'P_A3', 'P'}         );
    disp(tabl1);

    tabl2 = table(  {tableData.actParamsFileName}', [tableData.k_p_1]', [tableData.k_p_2]', ...
                    {tableData.radii_1}', {tableData.radii_2}', ...
                    'RowNames', {tableData.name}', ...
                    'VariableNames', {'actParamsFileName', 'k_p_1', 'k_p_2', 'radii_1', 'radii_2'}      );
    disp(tabl2);

    % Show another table with tensile forces etc
    tabl3 = table(  [tableData.dL_p_1_max]', [tableData.dL_p_2_max]', [tableData.dL_p_3_max]', ...
                    [tableData.F_p_1_max]', [tableData.F_p_2_max]', [tableData.F_p_3_max]', ...
                    [tableData.p_1_min]', [tableData.p_2_min]', [tableData.p_3_min]', ...
                    [tableData.p_1_max]', [tableData.p_2_max]', [tableData.p_3_max]', ...
                    'RowNames', {tableData.name}', ...
                    'VariableNames', {  'dL_p_1_max', 'dL_p_2_max', 'dL_p_3_max', ...
                                        'F_p_1_max', 'F_p_2_max', 'F_p_3_max', ...
                                        'p_1_min', 'p_2_min', 'p_3_min', ...
                                        'p_1_max', 'p_2_max', 'p_3_max'                 }               );
    disp(tabl3);


    %% Plots

    % Get variables
    gear_ratios     = [plot_data.gear_ratio];
    q_rotor_dot_1	= [plot_data.q_rotor_dot_1];
    q_rotor_dot_2	= [plot_data.q_rotor_dot_2];
    P_A12           = [plot_data.P_A12];
    P_A22           = [plot_data.P_A22];

    % Rotor velocity
    figure; hold on; grid on;
    plot(gear_ratios(1:5), q_rotor_dot_1(1:5), 'x--', 'Color', 'red');
    plot(gear_ratios(6:end), q_rotor_dot_1(6:end), 'o--', 'Color', 'red');
    plot(gear_ratios(1:5), q_rotor_dot_2(1:5), 'x--', 'Color', 'blue');
    plot(gear_ratios(6:end), q_rotor_dot_2(6:end), 'o--', 'Color', 'blue');
    xlabel('M2 gear ratio []');
    ylabel('Max M2 rotor speed [rad/s]');
    legend('q_{dot,1} mono', 'q_{dot,1} bi', 'q_{dot,2} mono', 'q_{dot,2} bi');
    title('M2 rotor speed over M2 gear ratio');

    % Power
    figure; hold on; grid on;
    plot(gear_ratios(1:5), P_A12(1:5), 'x--', 'Color', 'red');
    plot(gear_ratios(6:end), P_A12(6:end), 'o--', 'Color', 'red');
    plot(gear_ratios(1:5), P_A22(1:5), 'x--', 'Color', 'blue');
    plot(gear_ratios(6:end), P_A22(6:end), 'o--', 'Color', 'blue');
    xlabel('M2 gear ratio []');
    ylabel('Max M2 power [W]');
    legend('P_{A1,2} mono', 'P_{A1,2} bi', 'P_{A2,2} mono', 'P_{A2,2} bi');
    title('M2 power over M2 gear ratio');

end