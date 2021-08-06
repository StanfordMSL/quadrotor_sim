%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
% model = model_init('v1.0.0');  
model = model_init('iris');  

% Objective and Constraints
obj  = race_init('climb','gate_flat');

% Trajectory Horizon
t_hzn = 5;

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% % Generate QP Matrices
% QP_init(model.df.ndr);                       
% 
% % Generate Dynamics and Linearization Functions
% dyn_init(model,input_mode);      
% 
% % Generate Constraint Variables
% lagr_init(cost_mode,input_mode)
% motor_con_init(model,input_mode)
% gate_con_init(model,input_mode)

%% Trajectory Planning

traj = traj_init(obj,model,t_hzn,input_mode);

% Warm Start
traj = diff_flat(obj,model,traj,input_mode);
nominal_plot(traj.x_bar,obj.gt,10,'nice');

% Modifier for Debugging
% obj  = race_init('line','gate_right');
% nominal_plot(traj.x_bar,obj.gt,10,'persp');

% Full Constraint Optimization
traj = al_ilqr(traj,obj);
%c

%% Simulation

% MATLAB
log_M = matlab_sim(traj,obj,model,'none',input_mode,'bypass');
animation_plot(log_M,obj,model.map,'persp','show');

% % ROS
% log_R = gazebo_sim(traj,'body_rate');
% sim_compare(log_M,log_R)

%% Plot the States, Animate and Debug

% tol_motor = 2e3;
% tol_gate  = 2e-1;
% check_outer(log_M.con,tol_motor,tol_gate);

% animation_plot(log_M,obj,model.map,'back','show');
% br_debug(log_M.u_br)

%% Boneyard

% log = simulation(traj,obj,model,'none','pos_att','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

% mthrust_debug(log_M.u_mt,model)
