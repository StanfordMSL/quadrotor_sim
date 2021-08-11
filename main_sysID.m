addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Choose Simple Trajectory

% Options are: climb, drop, short/medium/long_line
mode = 'short_line';

%% Execute Position Trajectory Through ROS

% Initialize ROS Matlab Node
try
    rosnode list
catch
    rosinit('relay.local')
end

% Initialize ROS Parameters
node = ros.Node('/matlab_node');
pose_pub = ros.Publisher(node,'drone7/setpoint/position','geometry_msgs/PoseStamped');
pose_sub = ros.Subscriber(node,'drone7/mavros/local_position/pose');
u_sub = ros.Subscriber(node,'drone7/mavros/setpoint_raw/target_attitude');
pause(1);

% Target Point
pose = pose_sub.LatestMessage;
point0 = [pose.Pose.Position.X ; pose.Pose.Position.Y; pose.Pose.Position.Z];

switch mode
    case 'climb'
        offset = [0 ; 0 ; 1];
    case 'drop'
        offset = [0 ; 0 ; 0.5];
    case 'short_line'
        offset = [1 ; 0 ; 0];
    case 'medium_line'
        offset = [2 ; 0 ; 0];
    case 'long_line'
        offset = [3 ; 0 ; 0];
end
point = point0 + offset;

% Send to Point to Generate Data
[T,X,U] = send2point(pose_pub,pose_sub,u_sub,point);
D = [T ; X ; U];

% Save to csv
t_now = datetime;
t_now.Format = 'HHmmss';
save_folder = string(t_now);

mkdir('/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID',save_folder);
address = strcat('/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID','/',save_folder,'/',mode,'.csv');
csvwrite(address,D);

%% Execute Trajectory Through Simulation

save_folder = '185227';
mode = 'short_line';
address = strcat('/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID/',save_folder,'/',mode,'.csv');

D = csvread(address);
T = D(1,:);
X = D(2:14,:);
U = D(15:end,:);

% Initialize the nececssary parameters to run the matlab sim
model = model_init('v1.0.1');  
model = dyn_init(model,'body_rate');

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

for k = 1:N-1
    t_now = k/200;
    [~,idx] = min( abs(t_now-T(1,:)) );

    traj.u_br(:,k) = U(:,idx);
end

log_M = matlab_sim(traj,obj,model,'none','body_rate','bypass');
animation_plot(log_M,obj,model.map,'persp','show');

%% Analysis
% %% Verify Using Data on SD Card
% 
% % Insert Filename Here
% file_sd = '03_21_15';
% 
% % Extract Flight Data
% [T,X,U] = ulog2mat(file_sd);
% 
% % Find Take Off Point
% idx_to = find(U(1,:)>=0.29, 1);
% y = [T(idx_to) T(idx_to) ; -1000 1000];
% 
% figure(1)
% clf
% 
% subplot(4,1,1)
% plot(T,X(1,:));
% hold on
% plot(y(1,:),y(2,:),'r');
% ylim([-5 5])
% title('pos_x');
% 
% subplot(4,1,2)
% plot(T,X(2,:));
% hold on
% plot(y(1,:),y(2,:),'r');
% ylim([-5 5])
% title('pos_y');
% 
% subplot(4,1,3)
% plot(T,X(3,:));
% hold on
% plot(y(1,:),y(2,:),'r');
% ylim([0 3])
% title('pos_z');
% 
% subplot(4,1,4)
% plot(T,U(1,:));
% hold on
% plot(y(1,:),y(2,:),'r');
% ylim([0 1]);
% title('Normalized Thrust');