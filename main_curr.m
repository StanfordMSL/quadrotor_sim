%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('carlito','match','precise');  
% model = model_init('carlito_himo','match','precise');  
% model = model_init('iris','match','precise');  

% Objective and Constraints
% obj  = race_init('circuit_free',model.misc);
% obj  = race_init('circuit_obst',model.misc);
obj  = race_init('delt_line',model.misc);
% obj  = race_init('hover',model.misc);
% obj  = race_init('port_line',model.misc);
% obj  = race_init('vent_line',model.misc);
% obj  = race_init('slit_line_I',model.misc);
% obj = race_update(0);

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% Generate QP Matrices
QP_init(model.misc.ndr);                       

% Generate Dynamics and Linearization Functions
dyn_init(model,input_mode);      

% Generate Constraint Variables
lagr_init(cost_mode,input_mode)
conx_init(model,input_mode)
conu_init(model,input_mode)

%% Trajectory Planning

% Initialize Trajectory Structure
traj = traj_init(obj,model,input_mode);

% Warm Start Using Indirect Method
traj = diff_flat(obj,model,traj,input_mode);
% nominal_plot(traj.x_bar,obj,20,'persp');

% Full Constraint Optimizationy
[traj,~] = al_ilqr(traj,obj,999);
% nominal_plot(traj.x_bar,obj,20,'persp');

%% Simulation/Actual

% Modifications
traj_a = traj;
obj_a  = obj;

% MATLAB
log_M = matlab_sim(traj_a,obj_a,model,'al_ilqr',input_mode,'bypass');

% ROS -> Gazebo
log_G = ros_flight(traj_a,obj,'gazebo','single');

% % ROS -> Actual
% log_A = ros_flight(traj_a,obj,'actual','single');

%% Plot the States, Animate and Debug

% animation_plot(log_M,obj,'persp','show');

% sim_compare(traj,log_M,log_M)
% sim_compare(traj,log_M,log_G)
% sim_compare(traj,log_M,log_A)
