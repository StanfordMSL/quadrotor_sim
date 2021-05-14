clear; clc; 
addpath(genpath(pwd));

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('v1.0.0');  

% Objective and Constraints
obj  = obj_init('traj_gate');
map  = map_init('flightroom_medium');

% Order of Basis Function for QP
n_der = 15;             

% Cost Mode
cost = 'con_only';
% cost = 'min_input';
% cost = 'tracking';
% cost = 'terminal';
% cost = 'hover';

% Input Mode
input = 'direct';
% input = 'body_rate';
% input = 'body_rate_pid';

%% Pre-Computes (comment out after initial run to save time)

% dyn_lin_init(model,'direct');     % Generate Dynamics and Linearization Functions
% qp_init(n_der);                   % Generate QP Matrices

%% Warm Start

traj = diff_flat_ws(obj,map,model,n_der,'hide');

%% Full Constraint Optimization

% traj = al_ilqr(traj,obj,map,cost,input,model);

%% Simulation

log = simulation(traj,map,obj,model,'none','pos_att','bypass');
% log = simulation(traj,map,obj,model,'none','body_rate','bypass');
% log = simulation(traj,map,obj,model,'none','direct','bypass');

%% Plot the States, Animate and Debug

% des_err_debug(log);
animation_plot( log,obj,map,'nice','show');
% mthrust_debug(log.u_fmu,model)
