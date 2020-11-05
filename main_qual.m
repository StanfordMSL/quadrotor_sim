clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
% model        = model_init('v1.0.0');          % Initialize quadcopter
% model        = model_init('v1.0.1');          % Initialize quadcopter
model        = model_init('v1.1.1');          % Initialize quadcopter

% [N_traj,obj] = obj_init_qual('side gate');           % Initialize objectives
[N_traj,obj] = obj_init_qual('long slit');           % Initialize objectives
% [N_traj,obj] = obj_init_qual('drop gate');           % Initialize objectives

wts_db       = wts_init();                    % Initialize State and Input Cost Weights
targ         = targ_init('none');

% Initialize trajectory to hover at initial
traj   = traj_init(N_traj,obj.wp_arr(:,1),model.hover_u);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start
tic

% Diff. Flat Warm Start
traj = diff_flat_ws(traj,obj,model,'show');

% % iLQR Warm Start
% traj = direct_ws(traj,obj,wts_db,model,'show');

toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

log = simulation(traj,obj,wts_db,model,targ,'msl_lqr');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

mthrust_debug(log.u_fmu,model)
animation_plot(log,obj,targ,'nice','show');

% fast_animation_plot(log.x_act,obj,'persp')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

% % SimSync Test
% sim_sync_debug(log,traj);

%%

% SIMULATION CURRENTLY RUNNING IDENTICAL TO FP. change motor_transform (commented out 'actual'),
% simulation dynamic model ('actual') and model_init (hz_act)
