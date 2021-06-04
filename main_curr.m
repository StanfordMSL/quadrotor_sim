clear; clc; 
addpath(genpath(pwd));

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('v1.0.0');  

% Objective and Constraints
obj  = obj_init('line');
map  = map_init('slit_hard');

% Order of Basis Function for QP
n_der = 15;             

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% qp_init(n_der);                   % Generate QP Matrices

% dyn_lin_init(model,input_mode);        % Generate Dynamics and Linearization Functions

% lagr_init(cost_mode,input_mode)
% motor_con_init(input_mode,model)
% gate_con_init(map,input_mode,model)


%% Warm Start
tic
traj = diff_flat_ws(obj,map,model,n_der,'show');
toc
%% Full Constraint Optimization
tic
traj = al_ilqr(traj,obj,map);
toc
%% Simulation

% log = simulation(traj,obj,model,'none','pos_att','bypass');
log = simulation(traj,obj,model,'none','body_rate','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

%% Plot the States, Animate and Debug

tol_motor = 2e3;
tol_gate  = 2e-1;
check_outer(log.con,tol_motor,tol_gate);

animation_plot( log,obj,map,'nice','show');
% mthrust_debug(log.u_fmu,model)
