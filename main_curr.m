clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
model  = model_init('v1.0.1');  % Initialize quadcopter
obj    = obj_init('gate II');      % Initialize objectives
wts    = wts_init();            % Initialize State and Input Cost Weights
targ   = targ_init("none");     % Initialize target

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% % Diff. Flat Warm Start
% traj = df_init(wp,model,'yaw','show');

% iLQR Warm Start
traj = warm_start(obj,wts,601,model,'show');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Simulation
% log = simulation(nom,wp,model,wts,targ,'al_ilqr','ideal');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Plot the States and Animate
% % motor_plot(log,model)
% animation_plot(log,wp,targ,'persp','show');
