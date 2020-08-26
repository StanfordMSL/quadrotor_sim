clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters
N_seg = 501;                    % interrim way of feeding number of frames per segment.

% Base Parameters
model  = model_init('v1.0.1');  % Initialize quadcopter
obj    = obj_init('gate II',N_seg);      % Initialize objectives
wts_db = wts_init();            % Initialize State and Input Cost Weights
targ   = targ_init("none");     % Initialize target

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% % Diff. Flat Warm Start
% traj = df_init(wp,model,'yaw','show');

% iLQR Warm Start
traj = msl_ilqr_ws(obj,wts_db,model,N_seg,'show');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
log = simulation(traj,obj,wts_db,model,targ,'msl_lqr');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate
animation_plot(log,obj,targ,'persp','show');

%% Plot the States and Animate
% SimSync Test
sim_sync_debug(log,traj);

%%
% save('saves/test_II.mat','traj')
% save('saves/test_0.mat','traj')
%%

% SIMULATION CURRENTLY RUNNING IDENTICAL TO FP. change motor_transform (commented out 'actual'),
% simulation dynamic model ('actual') and model_init (hz_act)
