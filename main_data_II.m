%% Pre-Setup Setup (paths and ROS msg linking)

addpath(genpath(pwd));
% addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
% clear; clc; 
% rehash toolboxcache

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Model Parameters
model = model_init('carlito','match','precise');  
cost_mode = 'terminal';
input_mode = 'body_rate';

% Objective and Constraints
obj  = race_init('port_line',model.misc);

tol_motor = 1e-3;
tol_gate  = 1e-3;
%% Populate Objective Changer
step = 0.5;
[Y,Z] = meshgrid(-2.5:step:2.5,0.5:step:2.5);

%% Randomizer
rng('default');
s = rng;

%% Data Variable

% Chart (vanill AL-iLQR, vanilla Diff Flat, FALCon)
%         Original Traj Time | Min Traj Time | Solve Time
% tr1
% tr2
% etc...
N = size(Y,1)*size(Y,2);
data = zeros(N,3);

%% Trajectory Planning
t_lim = 10;

for k = 1:N
    feed = rand(2,1);
    q = [ feed(1) ;  feed(2);  0 ;  0];
    q = q./norm(q);
    obj.kf.gt(3,1) = Y(k);
    obj.kf.gt(4,1) = Z(k);
    obj.kf.gt(5:8,1) = q;
    obj.kf.fo(2,1,2) = Y(k);
    obj.kf.fo(3,1,2) = Z(k);

    pose_gt = obj.kf.gt(2:8,1);
    gt_dim  = obj.db(obj.kf.gt(1,1)).gt_dim;
    map     = obj.map.lim;
    
    % Initialize Trajectory Structure
    traj = traj_init(obj,model,input_mode);
    
    % Warm Start Using Indirect Method
    traj = diff_flat(obj,model,traj,input_mode);
  
    % Full Constraint Optimizationy
    [traj,flag] = al_ilqr_v2(traj,obj,t_lim);
    data(k,1) = traj.t_fmu(end);

    a = tic;
    [traj,N_t] = min_time_augment(traj,obj,obj.kf.x(:,1),10);
    data(k,2) = traj.t_fmu(end);    
    data(k,3) = toc(a);
%     nominal_plot(trFAL.x_br,obj,10,'persp');
    disp(['Finished Idx: ' num2str(k)]);

end

