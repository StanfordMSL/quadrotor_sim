addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Choose Simple Trajectory

% Options are: climb, drop, short/medium/long_line
% mode = 'short_line';
mode = 'slant_line';

%% Generate/Load Traj Data

% Generate
address = point2traj(mode);

% % Load
% % address = '/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID/194203/short_line.csv';
% address = '/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID/194248/medium_line.csv';
% D = csvread(address);
% T = D(1,:);
% X = D(2:14,:);
% X(1:3,:) = X(1:3,:) - X(1:3,1);     % keep things centered
% U = D(15:end,:);

%% Setup Required Parameters for Simulation

% Convert obj to what was simulated
obj  = race_init('climb','gate_flat');
obj.kf.x(:,1) = X(:,1);
obj.kf.x(:,2) = X(:,end);

% Convert the inputs to the traj format
N = ceil(200*T(1,end));
traj.t_fmu = 0:1/200:T(1,end);
traj.x_bar = X;
traj.T  = N;
traj.x_br = zeros(10,N);
traj.u_br = zeros(4,N-1);
traj.L_br = zeros(4,10,N-1);

%% Model Tuning

% Initialize the nececssary parameters to run the matlab sim
model = model_init('v1.0.1');  
model = dyn_init(model,'body_rate');

%% Execute Trajectory Through Simulation

for k = 1:N-1
    t_now = k/200;
    [~,idx] = min( abs(t_now-T(1,:)) );

    traj.u_br(:,k) = U(:,idx);
end

log_M = matlab_sim(traj,obj,model,'none','body_rate','bypass');
% animation_plot(log_M,obj,model.map,'persp','show');

% Analysis
%% Verify Using Data on SD Card

figure(2)
clf

subplot(4,2,1)
plot(T,X(1,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(1,:));
ylim([-5 5])
legend('actual','simulated')
title('pos_x');

subplot(4,2,3)
plot(T,X(2,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(2,:));
ylim([-5 5])
legend('actual','simulated')
title('pos_y');

subplot(4,2,5)
plot(T,X(3,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(3,:));
ylim([-1 3])
legend('actual','simulated')
title('pos_z');

subplot(4,2,7)
plot(T,U(1,:));
ylim([0 1]);
title('Normalized Thrust');

subplot(4,2,2)
plot(T,X(7,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(7,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q0');

subplot(4,2,4)
plot(T,X(8,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(8,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q1');

subplot(4,2,6)
plot(T,X(9,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(9,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q2');

subplot(4,2,8)
plot(T,X(10,:));
hold on
plot(log_M.t_fmu,log_M.x_fmu(10,:));
ylim([-1.1 1.1])
legend('actual','simulated')
title('q3');
