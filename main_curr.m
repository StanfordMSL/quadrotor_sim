clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters
N_traj = 501;                           % interim way of feeding number of frames per segment.

% Base Parameters
model  = model_init('v1.0.1');          % Initialize quadcopter
obj    = obj_init('gate Ib');           % Initialize objectives
wts_db = wts_init();                    % Initialize State and Input Cost Weights

% Initialize trajectory to hover at initial
traj   = traj_init(N_traj,obj.wp_arr(:,1),model.hover_u);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% % Diff. Flat Warm Start
% traj = df_init(wp,model,'yaw','show');

% iLQR Warm Start
traj = direct_ws(traj,obj,wts_db,model,'show');

% % % Saved Warm Start
% load saves/gate_II.mat

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
% 
% log = simulation(traj,obj,wts_db,model,targ,'msl_lqr');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Plot the States and Animate
% 
% animation_plot(log,obj,targ,'persp','show');
% % fast_animation_plot(log.x_act,obj,'persp')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Plot the States and Animate
% 
% % SimSync Test
% sim_sync_debug(log,traj);

%%

% SIMULATION CURRENTLY RUNNING IDENTICAL TO FP. change motor_transform (commented out 'actual'),
% simulation dynamic model ('actual') and model_init (hz_act)
