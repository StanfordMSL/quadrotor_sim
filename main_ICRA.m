%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% ROS Start Up

clear node
droneID = 'drone2';
gateID   = 'slot';
[node,subs,pubs,srvs,obj] = ros_start(droneID,gateID);

%% Initialize Model, Objective, Constraint and Misc. Parameters

% ICRA Initializations
model = model_init('carlito','match','precise');  
cost_mode = 'terminal';
input_mode = 'body_rate';

% Objective and Constraints
tic
obj  = race_ros(subs,obj,model.misc);
toc
%% Trajectory Planning

% Initialize Trajectory Structure
traj = traj_init(obj,model,input_mode);

% Warm Start Using Indirect Method
traj = diff_flat(obj,model,traj,input_mode);
nominal_plot(traj.x_bar,obj,20,'persp');
traj_df = traj;

% Full Constraint Optimizationy
tic
[traj,~] = al_ilqr(traj,obj,999);
toc
nominal_plot(traj.x_bar,obj,20,'top');

%% Simulation/Actual

% Modifications
traj_a = traj;
obj_a  = obj;

% MATLAB
log_M = matlab_sim(traj_a,obj_a,model,'al_ilqr',input_mode,'bypass');

% % ROS -> Gazebo
% log_G = ros_flight(traj_a,obj,'gazebo','single');

% ROS -> Actual
log_A = ros_flight(traj_a,subs,pubs,srvs);

%% Plot the States, Animate and Debug

% animation_plot(log_M,obj,'persp','show');

% sim_compare(traj,log_M,log_M)
% sim_compare(traj,log_M,log_G)
sim_compare(traj,log_M,log_A)
