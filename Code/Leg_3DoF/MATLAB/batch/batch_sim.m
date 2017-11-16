% batch_sim.m
% Batch-run a number of simulations with varying loads and/or actuation parameters
%#ok<*NASGU> %#ok<*UNRCH>

function [] = batch_sim()
    % Status
    disp('Starting batch simulation run...');
    timestamp = char(datetime(datetime, 'Format','yyyy-M-d_HH-mm-ss'));

    % Define actuation parameters and (additional) load
    actParamsFileNames      = { '', ... // noESB
                                'actuatorParams_monoarticulated.mat', ...
                                'actuatorParams_biarticulated.mat'          };
    legParamsFileNames      = { 'Leg_3DoF_design_noESB', ...
                                'Leg_3DoF_design_mono', ...
                                'Leg_3DoF_design_bi'        };
    loads                   = 0;%0:5:15;
    
    % Simulation parameters
    ref_use_random          = 0;        % Random reference (off/on)
    ref_random_dt           = 2;        % Random reference position intervals (speed)
    tspan                   = [0 20];   % Simulation timespan [[s] [s]]

    % Check the filenames first before we start to avoid crashing later
    for i=1:length(actParamsFileNames)
        actParamsFileName   = actParamsFileNames(i);
        if (~strcmp(actParamsFileName{:}, '') && ~exist(actParamsFileName{:}, 'file'))
            error(['ERROR: Actuation parameters file ''' actParamsFileName{:} ''' does not exist!']);
        end
    end
    
    % Check the LEG filenames first before we start to avoid crashing later
    for i=1:length(legParamsFileNames)
        legParamsFileName   = legParamsFileNames(i);
        if (~strcmp(legParamsFileName{:}, '') && ~exist(['params/' legParamsFileName{:}], 'file'))
            error(['ERROR: Leg parameters file ''' legParamsFileName{:} ''' does not exist!']);
        end
    end

    % Loop through the loads and actuation parameters and perform simulations
    for i=1:length(actParamsFileNames)
        for j=1:length(loads)        
            % Get actuation parameters filename and load
            actParamsFileName   = actParamsFileNames(i);
            legParamsFileName   = legParamsFileNames(i);
            load                = loads(j);

            % Status
            disp('-----------------------------------');
            disp('-- running simulation:');
            disp(['-- Actuation parameters file: ''' actParamsFileName{:} '''']);
            disp(['-- Leg parameters: ''' legParamsFileName{:} '''']);
            disp(['-- Load: ' num2str(load) ' kg']);
            disp('-----------------------------------');

            % Create simulator
            sim = Leg_3DoF_ACA_simulator(actParamsFileName{:}, legParamsFileName{:});
            
            % Change simulation timespan
            sim.params.tspan = tspan;

            % Modify leg load
            sim.model.leg.params.m3 = sim.model.leg.params.m3 + load;

            % Set random reference if enabled
            if (ref_use_random)
                % Save random reference as variable to use equal trajectory for
                % all runs
                if (~exist('const_ref', 'var'))
                    disp('Generating a new random reference...');

                    % Generate a new random reference
                    sim.model.ref_use_random(ref_use_random, sim.params.tspan, ref_random_dt);

                    % Save the generated ref for subsequent iterations
                    const_ref = sim.model.ref;
                else
                    % Set ref to previously saved
                    sim.model.ref = const_ref;

                    % Properly set initial states (see ref_use_random())
                    %q_0 	= const_ref.random.q_ref(:,1);
                    %q_d_0	= const_ref.random.q_d_ref(:,1);
                    %sim.model.setInitialStates(q_0, q_d_0);
                end
            end

            % Run simulation
            interactive = 0;
            sim.run(interactive);

            % Store resulting simulator object
            sim_data(i,j) = sim; %#ok<AGROW,NASGU>
        end
    end

    % Save data
    fileName = ['sim_batch_data_' timestamp '.mat'];
    disp(['Finished batch simulation run. Saving data as ''' fileName '''...']);
    save(fileName, 'sim_data', 'actParamsFileNames', 'legParamsFileNames', 'loads');

    % Finish
    disp('Completed.');
    
end
