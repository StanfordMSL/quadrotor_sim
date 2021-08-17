% addpath(genpath(pwd));
% addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
% rehash toolboxcache

%% Load Trajectory

load([pwd '/misc/tube_study/traj_001.mat']);
% load([pwd '/misc/tube_study/traj_002.mat']);

%% Tube Analysis

% rng('default');
% wt = [ randn(3,1) ; zeros(10,1)];
% wt = zeros(13,1);

% Unpack some stuff
Xact = traj.x_bar;
Xbar = traj.x_br;
Ubar = traj.u_br;
Larr = traj.L_br;
N = size(Xact,2);

% Prepare Container Variables
err = zeros(2,N);

% Roll the Dynamic Forward
br  = br_init(); 
FT_ext = zeros(6,1);
wt     = zeros(13,1);
u_now  = zeros(2,4,1);

for k = 1:N-1
%     del_x = [...
%         0.00001*randn(3,1) ;...
%         0.00001*randn(3,1) ;...
%         zeros(4,1)];
    del_x = zeros(10,1);
    
    u_now(1,:,1) = Ubar(:,k);
    u_now(2,:,1) = Ubar(:,k) + Larr(:,:,k)*del_x;

    for j = 1:2
        [u_wr,~] = br_ctrl(Xact(:,k),u_now(j,:,1)',br);
        u_mt =  w2m_est(u_wr);
        
        x_upd = quadcopter_est(Xact(:,k),u_mt,FT_ext,wt);
        
        err(j,k) = norm(x_upd-Xact(:,k+1));
    end
end

% Compare with Actual Update
figure(1)
clf
plot(err(1,:));
hold on
plot(err(2,:));
legend('open loop','closed loop')