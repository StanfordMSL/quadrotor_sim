%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('carlito','match','precise');  
% model = model_init('iris','match','precise');  

% Objective and Constraints
obj  = race_init('line','gate_right');

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid

%% Pre-Computes (comment out after initial run to save time)

% % Generate QP Matrices
% QP_init(model.misc.ndr);                       
% 
% % Generate Dynamics and Linearization Functions
% dyn_init(model,input_mode);      
% 
% % Generate Constraint Variables
% lagr_init(cost_mode,input_mode)
% motor_con_init(model,input_mode)
% gate_con_init(model,input_mode)

%% Trajectory Planning

% Initialize Trajectory Structure
traj = traj_init(obj,model,input_mode);

% Warm Start Using Indirect Method
traj = diff_flat(obj,model,traj,input_mode);
% nominal_plot(traj.x_bar,obj.gt,1,'persp');

% Full Constraint Optimizationy
[traj,~] = al_ilqr(traj,obj,999);
% nominal_plot(traj.x_bar,obj.gt,1,'persp');

%% Simulation/Actual

traj_a = traj;
N_sim = size(traj_a.x_bar,2);
k = 1;
while k < N_sim
    [traj_a,t_end] = min_time_augment(traj_a,obj,k,100);
    N_sim = round((t_end*200) + 1);
    
    k = k+200;
end
