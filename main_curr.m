clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters
N_seg = 501;       % interrim way of feeding number of frames per segment.

% Base Parameters
model  = model_init('v1.0.1');  % Initialize quadcopter
obj    = obj_init('gate II');      % Initialize objectives
wts    = wts_init(N_seg);            % Initialize State and Input Cost Weights
targ   = targ_init("none");     % Initialize target

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% % Diff. Flat Warm Start
% traj = df_init(wp,model,'yaw','show');

% iLQR Warm Start
traj = msl_ilqr_ws(obj,wts,model,N_seg,'hide');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
log = simulation(traj,obj,wts,model,targ,'msl_lqr');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate
animation_plot(log,obj,targ,'persp','show');
