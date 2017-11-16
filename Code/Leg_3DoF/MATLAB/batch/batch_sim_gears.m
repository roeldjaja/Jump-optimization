% batch_sim_gears.m
% Batch-run a number of simulations with varying gearbox ratios and actuation parameters

function [] = batch_sim_gears()
    % Status
    disp('Starting batch simulation run...');
    timestamp = char(datetime(datetime, 'Format','yyyy-M-d_HH-mm-ss'));

    % Define actuation parameters and loads
    actParamsFileNames     = {  'optimizationResults_monoarticulated.mat', ...
                                'optimizationResults_biarticulated.mat'      	};
    gear_ratios             = 1e4:1e4:5e4;
    ref_use_random          = 0;
    ref_random_dt           = 2; %#ok<NASGU>

    % Check the filenames first before we start to avoid crashing later
    for i=1:length(actParamsFileNames)
        actParamsFileName	= actParamsFileNames(i);
        if (~strcmp(actParamsFileName{:}, '') && ~exist(actParamsFileName{:}, 'file'))
            error(['ERROR: Parameters file ''' actParamsFileName{:} ''' does not exist!']);
        end
    end

    % Loop through the loads and actuation parameters and perform simulations
    for i=1:length(actParamsFileNames)
        for j=1:length(gear_ratios)        
            % Get actuation parameters filename and gearbox ratio
            actParamsFileName	= actParamsFileNames(i);
            gear_ratio          = gear_ratios(j);

            % Status
            disp(['Running simulation for parameter file ''' actParamsFileName{:} ''' and M2 gearing ratio of ' num2str(gear_ratio) '...']);

            % Create simulator
            sim = Leg_3DoF_ACA_simulator(actParamsFileName{:});

            % Change M2 gearing ratios
            sim.model.a1.params.r_m2 = gear_ratio;
            sim.model.a2.params.r_m2 = gear_ratio;
            sim.model.a3.params.r_m2 = gear_ratio;
            
            % TEST: Increase max ESB motor current
            %sim.model.a1.params.i_2_lim = 10;
            %sim.model.a2.params.i_2_lim = 10;
            %sim.model.a3.params.i_2_lim = 10;

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
                    q_0 	= const_ref.random.q_ref(:,1);
                    q_d_0	= const_ref.random.q_d_ref(:,1);
                    sim.model.setInitialStates(q_0, q_d_0);
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
    fileName = ['sim_gears_data_' timestamp '.mat'];
    disp(['Finished batch simulation run. Saving data as ''' fileName '''...']);
    save(fileName, 'sim_data', 'actParamsFileNames', 'gear_ratios');

    % Finish
    disp('Completed.');
    
end