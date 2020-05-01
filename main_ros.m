clear; clc; 
addpath(genpath(pwd));
lastwarn('');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Initialization

figure(1)
clf
des = plot3(0,0,0,'*');
hold on
act = plot3(0,0,0,'d');
xlim([-8.1 8.1]);
ylim([-3.2 3.2]);
zlim([-1 3]);
daspect([1 1 1])
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ROS Initialization
t_hzn = 3;

% rosshutdown
% setenv('ROS_MASTER_URI','http://localhost:11311')
% rosinit('http://localhost:11311');
% pause(1);

%%% ROS Initialization
[js_sub,pose_sub,twist_sub,m_cmd_pub] = ros_init();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('line');                         % Initialize mission
model  = model_init('simple v0.6','high-speed');    % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("none");                         % Initialize target

% iLQER Parameters
model.gamma = 0.00;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Warm Start the nominal trajectory for the iLQR
nom = df_init(wp,model,'yaw','hide');

%% TODO: FS_Trig and sim_key implementation into simulation function
% % % % Run the actual simulation
% % % fs_trig = 0;       % Trigger for motor cut;
% % % sim_key = 1;       % If true (1), we run as a simulation, else we are running in ROS
% % % % Fast Plot
% % % if (mod(sim_time,1/plot_hz) < tol)   
% % %     des.XData = wp.x(1,2);
% % %     des.YData = wp.x(2,2);
% % %     des.ZData = wp.x(3,2);
% % % 
% % %     act.XData = flight.x_act(1,k_act);
% % %     act.YData = flight.x_act(2,k_act);
% % %     act.ZData = flight.x_act(3,k_act);
% % % end

log = simulation(nom,wp,model,wts,targ,'ilqr');
