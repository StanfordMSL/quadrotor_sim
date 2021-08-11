%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('v1.0.1');  

% Objective and Constraints
obj  = race_init('default','gate_flat');

% Trajectory Horizon
t_hzn = 20;

% Cost Mode
cost_mode = 'terminal';      % || con_only || terminal || min_time || min_energy ||

% Input Mode
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid   

%% Pre-Computes (comment out after initial run to save time)

% Generate Dynamics and Linearization Functions
model = dyn_init(model,input_mode);     

%% Extract Flight Data

% Raw Extract
raw_att_act = csvread('misc/actual_flight_data/FlightTest02/03_21_15_vehicle_attitude_0.csv',1,0);
raw_pos_act = csvread('misc/actual_flight_data/FlightTest02/03_21_15_vehicle_local_position_0.csv',1,0);
raw_br_act  = csvread('misc/actual_flight_data/FlightTest02/03_21_15_vehicle_angular_velocity_0.csv',1,0);
raw_br_sp  = csvread('misc/actual_flight_data/FlightTest02/03_21_15_vehicle_rates_setpoint_0.csv',1,0);

% Processed Extract
N_act = size(raw_pos_act,1);
x_act = zeros(13,N_act);
R = [ 0 1 0 ; 1 0 0 ; 0 0 -1];

t_act = raw_pos_act(:,1)'./(1e6);
t_act = t_act-t_act(1);
x_act(1:3,:) = R*raw_pos_act(:,5:7)';
x_act(4:6,:) = raw_pos_act(:,11:13)';

for k = 1:N_act
    t_now = raw_pos_act(k,1);
    
     [~,idx] = min( abs(t_now-raw_att_act(:,1)) );
     x_act(7:10,k) = raw_att_act(idx,2:5)';
     
     [~,idx] = min( abs(t_now-raw_br_act(:,1)) );
    x_act(11:13,k) = raw_br_act(idx,3:5)';
end

traj = traj_init(obj,model,t_hzn,input_mode);
traj.type = 'body_rate';
traj.u_br = [-raw_br_sp(:,7)' ; R*raw_br_sp(:,2:4)'];

for k = 1:size(traj.u_br,2)
    if traj.u_br(1,k) < model.motor.c_hover
        traj.u_br(1,k) = model.motor.c_hover;
    end
    traj.u_br(2:4,k) = 0.00;

end

dt = (1e-6).*(raw_br_sp(2:end) - raw_br_sp(1:end-1)); % (timestamps are in microseconds)
traj.x_bar = x_act;


%% Simulate Dynamics Using Open Loop Inputs
log_M = matlab_sim(traj,obj,model,'none',input_mode,'bypass');

%%
figure(5)
clf

plot(t_act,x_act(3,:));
hold on
plot(log_M.t_act,log_M.x_act(3,:));
ylim([-1 3])

% animation_plot(log_M,obj,model.map,'persp','show');