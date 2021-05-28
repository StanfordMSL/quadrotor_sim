clear; clc; 
addpath(genpath(pwd));

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('v1.0.0');  

% Objective and Constraints
obj  = obj_init('traj_gate');
% map  = map_init('flightroom_wall_left');
map  = map_init('flightroom_hard');

% Order of Basis Function for QP
n_der = 15;             

% Cost Mode% || con_only || tracking || terminal || min_hover
cost_mode = 'terminal';      % || con_only || tracking || terminal || min_hover || min_input

% Input Mode
input_mode = 'body_rate';    % || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% dyn_lin_init(model,input_mode);        % Generate Dynamics and Linearization Functions
% % % % qp_init(n_der);                   % Generate QP Matrices

% al_ilqr_init(cost_mode,input_mode,obj,map,model);

% lagr_init(cost_mode,input_mode,obj)
% motor_con_init(input_mode,model)
% gate_con_init(map,input_mode,model)


%% Warm Start

traj = diff_flat_ws(obj,map,model,n_der,'show');

%% Full Constraint Optimization

traj = al_ilqr(traj,map);

%% Simulation

% log = simulation(traj,obj,model,'none','pos_att','bypass');
log = simulation(traj,obj,model,'none','body_rate','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

%% Plot the States, Animate and Debug

% des_err_debug(log);
animation_plot( log,obj,map,'nice','show');
% mthrust_debug(log.u_fmu,model)
