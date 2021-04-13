clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Quadcopter Model
model = model_init('v1.0.0');   

% Order of Basis Function (n_der-1)
n_der = 15;                     

%% Pre-Computes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Generate Dynamic Functions (Jacobian and Hessians)
% dyn_init(model,'act');
% dyn_init(model,'est');
% 
% % Generate QP Matrices
% qp_init(n_der);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Objective and Map Constraints
map = map_init('flightroom_easy');        % Initialize objectives
obj = obj_init('traj_easy');           % Initialize objectives
problem_plot(map,obj,'persp');

targ   = targ_init('none');       % Initialize Target (perching/grasping)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start
tic

% Diff. Flat Warm Start
traj = diff_flat_ws(obj,map,model,n_der,'hide');

% % iLQR Warm Start
% traj = direct_ws(obj,map,model,'show');

toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% log = simulation(traj,obj,wts_db,model,targ,'df');
log = simulation(traj,map,obj,model,targ,'df','br_ctrl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

mthrust_debug(log.u_fmu,model)
animation_plot(log,obj,map,targ,'nice','show');

% fast_animation_plot(log.x_act,obj,'persp')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

% % SimSync Test
% sim_sync_debug(log,traj);

% figure(1);
% clf
% plot3(squeeze(traj.f_out(1,1,:)),squeeze(traj.f_out(2,1,:)),squeeze(traj.f_out(3,1,:)))
