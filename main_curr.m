clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
model        = model_init('v1.0.1');    % Initialize quadcopter

% Generate Dynamic Functions (Jacobian and Hessians and Acc)
% dyn_init(model,'act');
% dyn_init(model,'est');

% [N_traj,obj] = obj_init('line',model);        % Initialize objectives
[N_traj,obj] = obj_init('side gate',model);   % Initialize objectives
% [N_traj,obj] = obj_init('drop gate',model);   % Initialize objectives
% [N_traj,obj] = obj_init('long slit',model);   % Initialize objectives

wts_db = wts_init();              % Initialize State and Input Cost Weights
targ   = targ_init('none');       % Initialize Target (perching/grasping)

% Initialize trajectory to hover at initial
traj   = traj_init(N_traj,obj.wp_arr(:,1),model.Ft_hover);

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

% log = simulation(traj,obj,wts_db,model,targ,'df');
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

err = HO_calc(traj);
