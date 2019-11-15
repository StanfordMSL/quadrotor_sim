clear; clc; 
addpath(genpath(pwd));
lastwarn('');

%     rosshutdown
%     setenv('ROS_MASTER_URI','http://localhost:11311')
%     rosinit('http://localhost:11311');
%     pause(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
t_hzn = 3;

est_hz = 100;      % State Estimator Time Counter
lqr_hz = 2;        % Low Rate Controller Sample Rate
con_hz = 40;       % High Rate Controller Sample Rate (lqr_dt*con_hz needs to be whole)
act_hz = 1000;      % Actual Dynamics Sample Rate
plot_hz = 10;      % Fast Plotting Sample Rate

sim_dt = 1/lcm(lcm(est_hz,con_hz),lcm(lqr_hz,act_hz));

t_est = 0:1/est_hz:t_hzn;
t_lqr = 0:1/lqr_hz:t_hzn;
t_con = 0:1/con_hz:t_hzn;
t_act = 0:1/act_hz:t_hzn; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation

%%% Map, Dynamics and Control Initialization
model  = model_init('simple vII',est_hz,lqr_hz,con_hz,act_hz); % Initialize Physics Model
fc     = fc_init(model,'ilqr');                         % Initialize Controller
wp     = wp_init('horizon',0,t_hzn,'no plot');         % Initialize timestamped keyframes
flight = flight_init(model,t_hzn,wp);                   % Initialize Flight Variables
targ   = targ_init("pigeon");                           % Iitialize target

%%% ROS Initialization
[l_pub,l_msg,L_pub,L_msg,js_sub] = ros_init(t_hzn,con_hz,flight.x_act(:,1),flight.u(:,1));

%%% Time Counters Initialization
k_est = 1;          % State Estimator Time Counter
k_lqr = 1;          % Low Rate Controller Time Counter
k_con = 1;          % High Rate Controller Time Counter
k_act = 1;          % Actual Dynamics Time Counter
k_wp  = 1;          % Waypoint Time Counter
tol = 1e-5;         % Tolerance to trigger various processes

%%% Contact Parameter Initialization
k_ct  = 1;
st_ct = 0;
dt_ct = 0.2;
N_ct  = round(dt_ct*act_hz); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

fs_trig = 0;       % Trigger for motor cut;
% Cold Start the nominal trajectory for the iLQR
nom = ilqr_init(flight.t_act(:,1),flight.x_act(:,1),wp,fc,model);
disp('[main]: Warm start complete! Ready to launch!');
disp('--------------------------------------------------')
pause;

while true
    sim_time = (k_act-1)*sim_dt;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if (mod(sim_time,1/est_hz) < tol)
        % Perfect Sensing (used for flight control)
        x_now = flight.x_act(:,k_act);
        flight.x_fc(:,k_est)  = x_now;
        
        k_est = k_est + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Rate Controller    
    if (mod(sim_time,1/lqr_hz) < tol) && (fs_trig == 0)
        
        % Update Active Trajectory
        data = receive(js_sub,1);
    
        wp.x(1,2) = data.Point.X;
        wp.x(2,2) = data.Point.Y;
        wp.x(3,2) = data.Point.Z;
    
        wp.x(1,3) = data.Point.X;
        wp.x(2,3) = data.Point.Y;
        wp.x(3,3) = data.Point.Z;
      
        % Update LQR params
        [nom, fs_trig] = hzn_ilqr(x_now,wp,nom,fc,model,t_hzn);
        k_lqr = k_lqr + 1;
        k_con = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Rate Controller    
    if (mod(sim_time,1/con_hz) < tol)
        if (fs_trig == 0)
            % Draw Out Motor Commands from u_bar computed by iLQR
            del_x = x_now-nom.x_bar(:,k_con);
            del_u = nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
            u  = nom.u_bar(:,k_con) + del_u;
            curr_m_cmd = wrench2m_controller(u,model);

            k_con = k_con + 1;
        else
            curr_m_cmd = zeros(4,1);
        end

        l_msg.Data = nom.l(:);
        send(l_pub,l_msg);
        
        L_msg.Data = nom.L(:);
        send(L_pub,L_msg);
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    if (mod(sim_time,1/act_hz) < tol)
%         check_ct = targ.pos - flight.x_act(1:3,k_act);
%         
%         if (norm(check_ct) < model.leg_l) && (st_ct == 0)
%             [FT_ext_arr,model,targ] = contact_func(model,targ,t_act(k_act),flight.x_act(:,k_act),check_ct,N_ct);
%             
%             FT_ext = FT_ext_arr(:,k_ct);
%             k_ct = k_ct + 1;
% 
%             st_ct = 1;
%             
%             disp('[main]: Contact triggered!');
%         elseif (k_ct <= N_ct) && (st_ct == 1)
%             FT_ext = FT_ext_arr(:,k_ct);
%             k_ct = k_ct + 1;
%         else
%             FT_ext = zeros(6,1);
%         end
        FT_ext = zeros(6,1);
        flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext,'actual');
        
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
