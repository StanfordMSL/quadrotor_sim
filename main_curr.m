%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
% model = model_init('carlito');  
model = model_init('iris');  

% Objective and Constraints
obj  = race_init('line','gate_right');

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% % Generate QP Matrices
% QP_init(model.misc.ndr);                       

% Generate Dynamics and Linearization Functions
model = dyn_init(model,input_mode);      

% % Generate Constraint Variables
% lagr_init(cost_mode,input_mode)
% motor_con_init(model,input_mode)
% gate_con_init(model,input_mode)

%% Trajectory Planning

% Initialize Trajectory Structure
traj = traj_init(obj,model,t_hzn,input_mode);

% Warm Start Using Indirect Method
traj = diff_flat(obj,model,traj,input_mode);

% Full Constraint Optimization
traj = al_ilqr(traj,obj);

%% Simulation/Actual

% MATLAB
log_M = matlab_sim(traj,obj,model,'none',input_mode,'bypass');

% ROS -> Gazebo
log_G = ros_flight(traj,'gazebo');

% % ROS -> Actual
% log_A = ros_flight(traj,'actual');

%% Plot the States, Animate and Debug

% animation_plot(log_M,obj,model.map,'persp','show');
% sim_compare(log_M,log_R)
% br_debug(log_M.u_br)
