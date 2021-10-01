%% Environment Setup (MATLAB paths and ROS msg linking)

addpath(genpath(pwd));
clear; clc; 

%% Load Parameters

% Quad Model Parameters
model = model_init('carlito','match','precise');

%% Time Parameters

tf = 5;
dt = model.est.dt;
N  = (tf/model.est.dt)+1;

%% Generate Quadcopter Dynamics

[x_dot,x_upd] = quad2D_gen(model);
% [x_dot,x_upd] = quad3D_gen(model);

%% Generate Objective Function

[Jk,JN] = obj2D_gen();
% [Jk,JN] = obj3D_gen();

%% Generate Constraint


%% Workspace

u0 = [0.0 ; 0.0 ; 0.0 ; 0.0];
x0 = [zeros(3,1) ; 1.0 ; zeros(3,1) ; zeros(3,1) ; 10 ; 10 ; 0];
% u0 = [0.0 ; 1.0];
% x0 = zeros(6,1);
tspan = [0 tf];

opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
[t_ode,x_ode] = ode45(@(t,x) quad3Dc(x,u0),tspan,x0,opts);
x_ode = x_ode';

t_eul = linspace(0,tf,N);
x_eul = zeros(13,N);
x_eul(:,1) = x0;
for k = 1:N-1
    x_eul(:,k+1) = quad3Dd(x_eul(:,k),u0);
end

x_comp = [x_ode(:,end) x_eul(:,end)];

figure(1)
clf

subplot(4,1,1)
plot(t_eul,x_eul(4,:));
hold on
plot(t_ode,x_ode(4,:));

subplot(4,1,2)
plot(t_eul,x_eul(5,:));
hold on
plot(t_ode,x_ode(5,:));

subplot(4,1,3)
plot(t_eul,x_eul(6,:));
hold on
plot(t_ode,x_ode(6,:));

subplot(4,1,4)
plot(t_eul,x_eul(7,:));
hold on
plot(t_ode,x_ode(7,:));
legend('eul','ode');
% 
% simple_plot(x',10,'persp');
% q = x(:,4:7);
% vecnorm(q')
% %% Pre-Computes (comment out after initial run to save time)
% 
% % Generate QP Matrices
% QP_init(model.misc.ndr);                       

% Objective Function
% obj_gen()

% % Generate Dynamics and Linearization Functions
%      
% 
% % Generate Constraint Variables
% lagr_init(cost_mode,input_mode)
% conx_init(model,input_mode)
% conu_init(model,input_mode)
% 
% %% Trajectory Planning
% 
% % Initialize Trajectory Structure
% traj = traj_init(obj,model,input_mode);
% 
% % Warm Start Using Indirect Method
% tic
% traj = diff_flat(obj,model,traj,input_mode);
% t_df = toc;
% % nominal_plot(traj.x_br,obj,10,'persp');
% traj_df = traj;
% 
% % Full Constraint Optimiyzationy
% tic
% [traj,~] = al_ilqr_v2(traj,obj,999);
% t_al = toc;
% traj_al = traj;
% 
% nominal_plot(traj.x_bar,obj,20,'persp');
% 
% %% Simulation/Actual
% 
% % Modifications
% traj_a = traj;
% obj_a  = obj;
% 
% % MATLAB
% log_M = matlab_sim(traj_a,obj_a,model,'al_ilqr',input_mode,'bypass');
% 
% % % ROS -> Gazebo
% % log_G = ros_flight(traj_a,obj,'gazebo','single');
% 
% % ROS -> Actual
% % log_A = ros_flight(traj_a,obj,'actual','single');
% % log_A = ros_flight_old(traj_a,obj,'actual','single');
% 
% %% Plot the States, Animate and Debug
% 
% % animation_plot(log_M,obj,'persp','show');
% 
% [traj_a,N_t] = min_time_augment(traj_a,obj,obj.kf.x(:,1),10);
% 
% % sim_compare(traj,log_M,log_M)
% % sim_compare(traj,log_M,log_G)
% % sim_compare(traj,log_M,log_A)
