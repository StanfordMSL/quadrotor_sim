clear; clc; 
addpath(genpath(pwd));
lastwarn('');

% rosshutdown
% setenv('ROS_MASTER_URI','http://localhost:11311')
% rosinit('http://localhost:11311');
% pause(1);
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
%% Time and Simulation Rate
tf = 3;
t_hzn = 3;

lqr_hz = 2;         % iLQR Update Rate
ctl_hz = 20;       % Control Law Switch Rate
fbc_hz = 20;       % Feedback Controller Rate
est_hz = ctl_hz;    % State Estimator Sample Rate
act_hz = 1000;      % Actual Dynamics Update Rate
plot_hz = 10;      % Fast Plotting Sample Rate

sim_dt = 1/act_hz;
sim_N  = tf/sim_dt;

t_est = 0:1/est_hz:tf;
t_lqr = 0:1/lqr_hz:tf;
t_ctl = 0:1/ctl_hz:tf;
t_fbc = 0:1/fbc_hz:tf;
t_act = 0:1/act_hz:tf; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation

%%% Map, Dynamics and Control Initialization
model  = model_init('simple v0.6',est_hz,lqr_hz,ctl_hz,fbc_hz,act_hz); % Initialize Physics Model
wts    = wts_init();                     % Initialize Controller
wp     = wp_init('horizon',tf,'no plot'); % Initialize timestamped keyframes
flight = logger_init(tf,wp,act_hz,fbc_hz);     % Initialize Flight Variables
targ   = targ_init("none");          % Initialize target

%%% Time Counters Initialization
k_est = 1;              % State Estimator Time Counter
k_lqr = 1;              % iLQR Update Time Counter
k_ctl = 1;              % Control Law Switch Time Counter
k_fbc = 1;              % Feedback Control Time Counter
k_act = 1;              % Actual Dynamics Time Counter
k_wp  = 1;              % Waypoint Time Counter
tol   = 1e-1*sim_dt;    % Tolerance to trigger various processes

%%% Contact Parameter Initialization
k_ct  = 1;                   % Contact Force Time Counter
st_ct = 0;                   % Contact state (0 = no contact, 1 = contact/post-contact)
dt_ct = 0.2;                 % Duration of Contact Force
N_ct  = round(dt_ct*act_hz); % Number of actual dynamics frames

%%% ROS Initialization
[js_sub,pose_sub,twist_sub,m_cmd_pub] = ros_init();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
fs_trig = 0;       % Trigger for motor cut;
sim_key = 1;       % If true (1), we run as a simulation, else we are running in ROS

% Cold Start the nominal trajectory for the iLQR
nom = df_init(wp,model,'yaw');
disp('[main]: Warm start complete! Ready to launch!');
disp('--------------------------------------------------')
pause;

while true
    sim_time = (k_act-1)*sim_dt;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if (mod(sim_time,1/est_hz) < tol)
        x_now = flight.x_act(:,k_act);
        flight.x_fc(:,k_est)  = x_now;
       
        k_est = k_est + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Rate Controller    
    if (mod(sim_time,1/lqr_hz) < tol) && (fs_trig == 0)
        
        % Update Active Trajectory
        pose_rx = receive(js_sub,1);
    
        wp.x(1,2) = pose_rx.Point.X;
        wp.x(2,2) = pose_rx.Point.Y;
        wp.x(3,2) = pose_rx.Point.Z;
    
        % Update LQR params
        [nom, fs_trig] = hzn_ilqr(x_now,wp,nom,wts,model);
        k_lqr = k_lqr + 1;
        k_con = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Rate Controller    
    if (mod(sim_time,1/fbc_hz) < tol)
        if (fs_trig == 0)
            % Draw Out Motor Commands from u_bar computed by iLQR
            del_x = x_now-nom.x_bar(:,k_con);
            del_u = nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
            u  = nom.u_bar(:,k_con) + del_u;
            curr_m_cmd = wrench2m_controller(u,model);

            % Send out the ROS message
            msg = rosmessage(m_cmd_pub);
            feed = [curr_m_cmd ; zeros(4,1)];
            msg.Data = feed;
            send(m_cmd_pub,msg);
            % Update Count
            k_con = k_con + 1;
        else
            curr_m_cmd = zeros(4,1);
        end        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    if (mod(sim_time,1/act_hz) < tol)
        if sim_key == 0
            % Carry On
        else
            FT_ext = zeros(6,1);
            flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext,'actual');
        end
        
        k_act = k_act + 1;
    end
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Fast Plot
    if (mod(sim_time,1/plot_hz) < tol)   
        des.XData = wp.x(1,2);
        des.YData = wp.x(2,2);
        des.ZData = wp.x(3,2);
    
        act.XData = flight.x_act(1,k_act);
        act.YData = flight.x_act(2,k_act);
        act.ZData = flight.x_act(3,k_act);
    end
end
