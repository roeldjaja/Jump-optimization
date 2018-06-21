% comp_showResults.m

% Status
disp('Showing results for noESB, mono and/or bi cases..');


%% noESB
if (confirm('Show noESB results? [Y/n]', 1))
    load('comp_noESB.mat');
    opt = Leg_3DoF_ACA_jumpref_optimizer();
    opt.plot_matdata();

    disp('Press ENTER to continue..');
    pause;
end


%% Monoarticulated
if (confirm('Show mono results? [Y/n]', 1))
    load('comp_mono.mat');
    opt = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_monoarticulated.mat', 'Leg_3DoF_design_mono');
    opt.plot_matdata();

    disp('Press ENTER to continue..');
    pause;
end


%% Biarticulated
if (confirm('Show bi results? [Y/n]', 1))
    load('comp_bi.mat');
    opt = Leg_3DoF_ACA_jumpref_optimizer('actuatorParams_biarticulated.mat', 'Leg_3DoF_design_bi');
    opt.plot_matdata();
end

disp('Finished.');
