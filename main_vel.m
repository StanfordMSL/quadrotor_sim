clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Quadcopter Model
model = model_init('v1.0.0');  

n_der = 15;             % Order of Basis Function for QP
% qp_init(n_der);        % Generate QP Matrices

% Objective and Constraints
obj  = obj_init('massless');
map  = map_init('default');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start
tic

% Diff. Flat Warm Start
traj = diff_flat_ws(obj,map,model,n_der,'hide');

% % Full Constraint Resolution
% traj = al_ilqr(traj,obj,map,model);

toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% log = simulation(traj,obj,wts_db,model,targ,'df');
log = simulation(traj,map,obj,model,'df','pos_att');
% log = simulation(traj,map,obj,model,targ,'df','open_loop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

des_err_debug(log);
animation_plot(log,obj,map,'top','show');
% mthrust_debug(log.u_fmu,model)
