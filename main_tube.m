addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
% rehash toolboxcache

%% Load Trajectory

% % lqr.Qn = [10.0.*ones(3,1) ; 0.01.*ones(3,1) ; 0.1.*ones(4,1)];
% % lqr.QN = [100.0.*ones(3,1) ; 0.01.*ones(3,1) ; 0.1.*ones(4,1)];
% load([pwd '/misc/tube_study/traj_001.mat']);        

% lqr.Qn = [10.0.*ones(3,1) ; 0.01.*ones(3,1) ; 0.5.*ones(4,1)];
% lqr.QN = [10.0.*ones(3,1) ; 0.01.*ones(3,1) ; 0.5.*ones(4,1)];
load([pwd '/misc/tube_study/traj_002.mat']);        

obj  = race_init('line','empty');

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
err = zeros(2,N-1);

% Roll the Dynamic Forward
br  = br_init(); 
wt     = zeros(13,1);
u_now  = zeros(4,1,2);
   
XSim = zeros(13,N,2);
XSim(:,1,1) = Xact(:,1);
XSim(:,1,2) = Xact(:,1);

for k = 1:N-1  
    FT_ext = [0.5.*randn(3,1) ; zeros(3,1)];

    u_now(:,1,1) = Ubar(:,k);

    del_x = XSim(1:10,k,2)-Xbar(:,k);
    u_now(:,1,2) = Ubar(:,k) + Larr(:,:,k)*del_x;

    for j = 1:2
        [u_wr,~] = br_ctrl(XSim(:,k,j),u_now(:,1,j),br);
        u_mt =  w2m_est(u_wr);
        
        XSim(:,k+1,j) = quadcopter_est(XSim(:,k,j),u_mt,FT_ext,wt);

        err(j,k) = norm(XSim(:,k+1,j)-Xact(:,k+1));
    end
end

%% Compare with Actual Update

% nominal_plot(XSim(:,:,2),obj.gt,20,'persp');

figure(1)
clf
plot(err(1,:));
hold on
plot(err(2,:));
legend('open loop','closed loop')
ylim([-0.1 1])

compare = sum(err,2)