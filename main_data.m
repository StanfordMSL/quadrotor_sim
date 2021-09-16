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
step = 0.2;
[Y,Z] = meshgrid(-2.5:step:2.5,0.5:step:3.0);

%% Randomizer
rng('default');
s = rng;

%% Data Variable

% Chart (vanill AL-iLQR, vanilla Diff Flat, FALCon)
%         Solve Time  Constraint Satisfied 
% tr1
% tr2
% etc...
N = size(Y,1)*size(Y,2);
data = zeros(N,2,3);

%% Trajectory Planning
t_lim = 10;

for k = 1:N
%     feed = rand(2,1);
%     q = [ feed(1) ;  0 ;  0 ;  feed(2)];
%     q = q./norm(q);
    obj.kf.gt(3,1) = Y(k);
    obj.kf.gt(4,1) = Z(k);
%     obj.kf.gt(5:8,1) = q;
    obj.kf.fo(2,1,2) = Y(k);
    obj.kf.fo(3,1,2) = Z(k);

    pose_gt = obj.kf.gt(2:8,1);
    gt_dim  = obj.db(obj.kf.gt(1,1)).gt_dim;
    map     = obj.map.lim;
    
    % Initialize Trajectory Structure
    traj = traj_init(obj,model,input_mode);
    
    % Pure ALiLQR
    tAL = tic;
    [tr,flag] = al_ilqr(traj,obj,t_lim);
    TAL = toc(tAL);
    data(k,1,1) = TAL;
    hit = norm(tr.x_br(1:3,end)-obj.kf.x(1:3,end));
    if ((hit < 0.1) && (flag ==0))
        data(k,2,1) = 1;
    end
    
    % Warm Start Using Indirect Method
    tDF = tic;
    traj = diff_flat(obj,model,traj,input_mode);
    con  = con_calc(traj.x_br,traj.u_br,pose_gt,gt_dim,map);
    flag = check_outer(con,tol_motor,tol_gate);
    TDF = toc(tDF);
    data(k,1,2) = TDF;
    data(k,2,2) = flag;
    
    % Full Constraint Optimizationy
    tFAL = tic;
    [trFAL,flag] = al_ilqr_v2(traj,obj,t_lim);
    TFAL = toc(tFAL);
    data(k,1,3) = TFAL;
    hit = norm(trFAL.x_br(1:3,end)-obj.kf.x(1:3,end));
    if ((hit < 1.0) && (flag ==0))
        data(k,2,3) = 1;
    end
%     nominal_plot(trFAL.x_br,obj,10,'persp');
    disp(['Finished Idx: ' num2str(k)]);

end

