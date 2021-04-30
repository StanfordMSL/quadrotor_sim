clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Quadcopter Model
model = model_init('v1.1.0');  

% % Pre-Compute (comment out after first run to save time)
% precompute(model);

% Objective and Constraints
obj  = obj_init('crescent');
map  = map_init('default');
targ = targ_init('none');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start
tic

% Diff. Flat Warm Start
n_der = 15; % Order of Basis Function
traj = diff_flat_ws(obj,map,model,n_der,'hide');

% % iLQR Warm Start
% traj = direct_ws(obj,map,model,'show');

toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% log = simulation(traj,obj,wts_db,model,targ,'df');
log = simulation(traj,map,obj,model,targ,'df','br_ctrl');
% log = simulation(traj,map,obj,model,targ,'df','open_loop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

mthrust_debug(log.u_fmu,model)
animation_plot(log,obj,map,targ,'nice','show');

