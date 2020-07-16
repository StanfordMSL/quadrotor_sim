clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
model  = model_init('v1.0.1');  % Initialize quadcopter
wp     = wp_init('climb');      % Initialize mission
wts    = wts_init();            % Initialize State and Input Cost Weights
targ   = targ_init("none");     % Initialize target

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% % Diff. Flat Warm Start
% nom = df_init(wp,model,'yaw','show');

% iLQR Warm Start
[wp,wts,nom,model] = warm_start(wp,wts,model);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Simulation
% log = simulation(nom,wp,model,wts,targ,'al_ilqr','ideal');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Plot the States and Animate
% % motor_plot(log,model)
% animation_plot(log,wp,targ,'persp','show');
