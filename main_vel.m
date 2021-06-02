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

log = simulation(traj,obj,model,'none','pos_att','bypass');
% log = simulation(traj,obj,model,'none','body_rate','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

animation_plot(log,obj,map,'top','show');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save the Flat Output in Pos/Vel csv

fout2csv(log.t_fmu,traj.f_out)
