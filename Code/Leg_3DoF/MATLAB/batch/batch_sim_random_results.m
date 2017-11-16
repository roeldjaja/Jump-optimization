% batch_sim_random_results.m
% [] = batch_sim_random_results(filenamePattern, filenames)
% Process and optionally plot the result of the batch simulations using
% random references

function [] = batch_sim_random_results(filenamePattern, filenames)
    % Default arguments
    if (~exist('filenamePattern', 'var'))
        filenamePattern = 'sim_batch_*.mat';
        disp(['Set default filename pattern ''' filenamePattern '''...']);
    end
    
    % Process filename pattern *or* cell array of filenames
    if (~strcmp(filenamePattern, ''))
        % List directory, grabbing files that match the pattern
        list = dir(filenamePattern);
        for i=1:length(list)
           filenames{i} = list(i).name; 
        end
    else
       % Check if files exist first
        if (~exist('filenames', 'var') || isempty(filenames))
            error('No filenames given!');
        end
        for i=1:length(filenames)
            if (~exist(filenames{i}, 'file'))
                error(['File ' num2str(i) ' ''' filename ''' does not exist!']);
            end
        end
    end
    
    % Params
    ignoreStartTime = 20.0; % How much time to ignore at the start of the motion [s]
    
    % mean energy consumptions
    meanNoESB   = 0;
    meanMono    = 0;
    meanBi      = 0;
    noESBCount  = 0;
    monoCount   = 0;
    biCount     = 0;
    
    % Prepare plotting figures
    figure(1); grid on; hold on;
%     figure(2); grid on; hold on;
    figure(3); grid on; hold on;

    %% Go through all files
    for i=1:length(filenames)
        % Load data
        disp(['Loading file ''' filenames{i} '''...']);
        clear sim_data actParamsFileNames loads
        load(filenames{i});
        
        % Go through all simulators
        for j=1:length(sim_data)
            % Get simulator, actuator parameters filename and load
            sim         = sim_data(j);
            %actParamsFileName = sim.model.actParamsFileName;

            % Time
            t           = sim.data.t;
            Ts      	= sim_data(1).params.Ts;
            
            % Get motor powers
            P_A11   	= sim.data.y_a1(sim.model.a1.outputIdx_P_1,:);
            P_A12      	= sim.data.y_a1(sim.model.a1.outputIdx_P_2,:);
            P_A21    	= sim.data.y_a2(sim.model.a2.outputIdx_P_1,:);
            P_A22       = sim.data.y_a2(sim.model.a2.outputIdx_P_2,:);
            P_A31    	= sim.data.y_a3(sim.model.a3.outputIdx_P_1,:);
            P_A32     	= sim.data.y_a3(sim.model.a3.outputIdx_P_2,:);
            
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
            P_filt      = filter(ones(1,windowSize) / windowSize, 1, P);
            
            % Get cumulative electrical energy consumption
            E_A1        = sim.data.E_A1;
            E_A2        = sim.data.E_A2;
            E_A3        = sim.data.E_A3;
            E           = sim.data.E;
            
            % Plot
            figure(1);
            %scatter(j, P_A1_filt(end));
            %scatter(j, P_A2_filt(end));
            %scatter(j, P_A3_filt(end));
            scatter(j, P_filt(end));

            % Joint motion
            q_leg     	= sim.data.x_leg(1:3,:); %#ok<*UNRCH>
            
            % Net joint torque
            tau      	= [	sim.data.y_a1(sim.model.a1.outputIdx_tau(sim.model.a1.topology.idx),:); ...
                           	sim.data.y_a2(sim.model.a2.outputIdx_tau(sim.model.a2.topology.idx),:); ...
                           	sim.data.y_a3(sim.model.a3.outputIdx_tau(sim.model.a3.topology.idx),:)  ];
            
            % ESB torque
            tau_p     	= [	sim.data.y_a1(sim.model.a1.outputIdx_tau_p(sim.model.a1.topology.idx),:); ...
                          	sim.data.y_a2(sim.model.a2.outputIdx_tau_p(sim.model.a2.topology.idx),:); ...
                          	sim.data.y_a3(sim.model.a3.outputIdx_tau_p(sim.model.a3.topology.idx),:)  ];
            % PB torque
            tau_pb    	= [ sim.data.y_a1(sim.model.a1.outputIdx_tau_pb(sim.model.a1.topology.idx),:); ...
                          	sim.data.y_a2(sim.model.a2.outputIdx_tau_pb(sim.model.a2.topology.idx),:); ...
                          	sim.data.y_a3(sim.model.a3.outputIdx_tau_pb(sim.model.a3.topology.idx),:)  ];
            
            % Get index at which to start the plot to ignore the startup
            % effects
            iIdx = max(1, ignoreStartTime / Ts);
            
            % Get whether this was using random ref, for line width
            lineWidth = 1;
            if (sim.model.ref.use_random == 0)
                lineWidth = 2;
            end
            
            % Plot
%             figure(2);
%             if (j == 1)
%                 % noESB
%                 h1 = plot((180/pi)*q_leg(1,iIdx:end)', tau(1,iIdx:end)', 'Color', [0.3 0.3 1.0], 'LineWidth', lineWidth);
%                 h2 = plot((180/pi)*q_leg(2,iIdx:end)', tau(2,iIdx:end)', 'Color', [1.0 0.4 0.4], 'LineWidth', lineWidth);
%             elseif (j == 2)
%                 % Monoarticulated
%                 h3 = plot((180/pi)*q_leg(1,iIdx:end)', tau_pb(1,iIdx:end)', '--', 'Color', [0.6 0.6 1.0], 'LineWidth', lineWidth);
%                 h4 = plot((180/pi)*q_leg(2,iIdx:end)', tau_pb(2,iIdx:end)', '--', 'Color', [1.0 0.6 0.6], 'LineWidth', lineWidth);
%             elseif (j == 3)
%                 % Biarticulated
%                 h5 = plot((180/pi)*q_leg(1,iIdx:end)', tau_pb(1,iIdx:end)', 'Color', [0.6 0.6 1.0], 'LineWidth', lineWidth);
%                 h6 = plot((180/pi)*q_leg(2,iIdx:end)', tau_pb(2,iIdx:end)', 'Color', [1.0 0.6 0.6], 'LineWidth', lineWidth);
%             else
%                 warning('j not in range 1..3');
%             end
            
            % Plot
            figure(3);
            if (j == 1)
                % noESB
                h7 = plot(t, E, 'Color', [0.3 0.3 1.0]);
                meanNoESB   = meanNoESB + E(end) - E(iIdx);
                noESBCount  = noESBCount+1;
            elseif (j == 2)
                % Monoarticulated
                h8 = plot(t, E, 'Color', [1.0 0.4 0.4]);
                meanMono    = meanMono + E(end) - E(iIdx);
                monoCount   = monoCount+1;
            elseif (j == 3)
                % Biarticulated
                h9 = plot(t, E, 'Color', [0.3 0.8 0.3]);
                meanBi      = meanBi + E(end) - E(iIdx);
                biCount     = biCount+1;
            else
                warning('j not in range 1..3');
            end
            
            % Draw now
            drawnow;
            
        end
        
    end
    
    % Calculate correct means
    meanNoESB   = meanNoESB / noESBCount;
    meanMono    = meanMono / monoCount;
    meanBi      = meanBi / biCount;
    
    % Plot properties
    figure(1);
    setYAxis;
    title('Power use per configuration (at end)');
    xlabel('Configuration (1=noESB, 2=mono, 3=bi)');
    ylabel('Power [W]');
    
%     figure(2);
%     setYAxis;
%     title('Joint torques over joint positions for random motions');
%     xlabel('Joint angle [deg]');
%     ylabel('Torque [Nm]');
%     legend( [h1, h2, h3, h4, h5, h6], ...
%             'Ankle (q1)', 'Knee (q2)', ...
%             'Ankle mono', 'Knee mono', ...
%             'Ankle bi', 'Knee bi'           );
        
    figure(3);
    setYAxis;
    title(['Cum. energy consump. p/ cfg. (ignoring first ' num2str(ignoreStartTime) ' s)']);
    xlabel('Time [s]');
    ylabel('Energy [J]');
    legend( [h7, h8, h9], ...
            ['noESB (mean ' num2str(meanNoESB,4) ' J)'],  ...
            ['mono. (mean ' num2str(meanMono,4) ' J, ' num2str(meanNoESB/meanMono,3) 'x less)'],    ...
            ['bi. (mean ' num2str(meanBi,4) ' J, ' num2str(meanNoESB/meanBi,3) 'x less)']           );
    
end

