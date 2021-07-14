addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache
%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('v1.0.1');  
% model = model_init('iris');  

% Objective and Constraints
obj  = obj_init('line');
map  = map_init('empty');

% Order of Basis Function for QP
n_der = 15;             

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% Generate QP Matrices
QP_init(n_der);                       

% Generate Dynamics and Linearization Functions
dyn_init(model,input_mode);      

% Generate Constraint Variables
lagr_init(cost_mode,input_mode)
motor_con_init(input_mode,model)
gate_con_init(map,input_mode,model)

%% Trajectory Planning

% Warm Start
traj = diff_flat(obj,map,model,n_der,'show');

% Full Constraint Optimization
traj = al_ilqr(traj,obj,map);

%% Simulation

% MATLAB
log_M = matlab_sim(traj,obj,model,'none','body_rate','bypass');
animation_plot( log_M,obj,map,'nice','show');

% ROS
% log_R = gazebo_sim(traj,'body_rate');
% sim_compare(log_M,log_R)

%% Plot the States, Animate and Debug

% tol_motor = 2e3;
% tol_gate  = 2e-1;
% check_outer(log_M.con,tol_motor,tol_gate);

% animation_plot( log_M,obj,map,'nice','show');
% br_debug(log_M.u_br)

%% Boneyard

% log = simulation(traj,obj,model,'none','pos_att','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

% mthrust_debug(log_M.u_mt,model)
