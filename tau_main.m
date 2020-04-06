clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization

% Tau Variables
tau_dot = [0.6,0.8,0.9]';
x0      = [-3 ; 0 ; 1 ; 1 ; 0 ; -1 ; 1 ; zeros(6,1)]; % WITH TARGET AS ORIGIN
dt      = 0.5;

% Data Variables
n_td  = size(tau_dot,1);        % number of tau_dots tested
n_x   = zeros(n_td,1);          % frame count sizes (since vary tau_dot changes total flight time)
n_wp  = zeros(n_td,1);
vel_f = zeros(n_td,1);          % final velocities
x_tau = zeros(13,8000,n_td);    % tau trajectories (in full state form)
wp_tau = zeros(13,800,n_td);
%% Simulation

for k = 1:n_td
    % simulate
    [x_tau_curr,wp_tau_curr] = tau_sim(tau_dot(k),x0,dt);
    
    % calculate and store final velocity
    vel_f(k,1) = round(norm(x_tau_curr(4:6,end)),2);    
    
    % store x frame count
    n_x_curr = size(x_tau_curr,2);
    n_x(k,1) = n_x_curr;
    
    % store wp frame count
    n_wp_curr = size(wp_tau_curr,2);
    n_wp(k,1) = n_wp_curr;
    
    % store trajectory 
    x_tau(:,1:n_x_curr,k) = x_tau_curr;
    wp_tau(:,1:n_wp_curr,k) = wp_tau_curr;
end

%% Plot
tau_plot(x_tau,wp_tau,vel_f,n_x,n_wp,tau_dot,'show')

