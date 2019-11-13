clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time and Simulation Rate
tf = 3;

est_hz = 200;       % State Estimator Time Counter
lqr_hz = 2;        % Low Rate Controller Sample Rate
con_hz = 200;       % High Rate Controller Sample Rate
act_hz = 1000;      % Actual Dynamics Sample Rate

sim_dt = 1/lcm(lcm(est_hz,con_hz),lcm(lqr_hz,act_hz));
sim_N  = tf/sim_dt;

t_est = 0:1/est_hz:tf;
t_lqr = 0:1/lqr_hz:tf;
t_con = 0:1/con_hz:tf;
t_act = 0:1/act_hz:tf; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation

%%% Map, Dynamics and Control Initialization
model  = model_init('simple vII',est_hz,con_hz,act_hz); % Initialize Physics Model
fc     = fc_init(model,'ilqr');                         % Initialize Controller
wp     = wp_init('targeted',0,tf,'no plot');              % Initialize timestamped keyframes
flight = flight_init(model,tf,wp);                      % Initialize Flight Variables
targ   = targ_init("pigeon");                           % Iitialize target

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

% Cold Start the nominal trajectory for the iLQR
nom = ilqr_init(flight.t_act(:,1),flight.x_act(:,1),wp,fc,model);
disp('[main]: Warm start complete! Ready to launch!');
disp('--------------------------------------------------')
pause;

for k = 1:sim_N
    sim_time = (k-1)*sim_dt;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if (abs(t_est(k_est)-sim_time) < tol) && (k_est <= tf*est_hz)
        % Perfect Sensing (used for flight control)
        t_now = t_est(k_est);
        x_now = flight.x_act(:,k_act);
        flight.x_fc(:,k_est)  = x_now;

%         %%%%%%%%%%%%%%%%%%%%
%         % YOLO UKF Test
%         t_now = t_est(k_est);
%         [sv,mu_prev] = yolo_ukf(sv,flight,k_est,t_now,initial_bb,camera,qtmp,qm,ukf_prms,mu_curr,mu_prev,sig_curr,model);
%         %%%%%%%%%%%%%%%%%%%%
        k_est = k_est + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Rate Controller    
    if (abs(t_lqr(k_lqr)-sim_time) < tol) && (k_lqr <= tf*lqr_hz)
        % Update LQR params
        nom = ilqr(t_now,x_now,wp,nom,fc,model);
        k_lqr = k_lqr + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Rate Controller    
    if (abs(t_con(k_con)-sim_time) < tol) && (k_con <= tf*con_hz)
        % Draw Out Motor Commands from u_bar computed by iLQR
        del_x = x_now-nom.x_bar(:,k_con);
        del_u = nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
        u  = nom.u_bar(:,k_con) + del_u;
        curr_m_cmd = wrench2m_controller(u,model);
        
        % Log State Estimation and Control
        flight.m_cmd(:,k_con) = curr_m_cmd;       
        
        k_con = k_con + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    if (abs(t_act(k_act)-sim_time) < tol)
        check_ct = targ.pos - flight.x_act(1:3,k);
        
        if (norm(check_ct) < model.leg_l) && (st_ct == 0)
            [FT_ext_arr,model,targ] = contact_func(model,targ,t_act(k_act),flight.x_act(:,k_act),check_ct,N_ct);
            
            FT_ext = FT_ext_arr(:,k_ct);
            k_ct = k_ct + 1;

            st_ct = 1;
            
            disp('[main]: Contact triggered!');
        elseif (k_ct <= N_ct) && (st_ct == 1)
            FT_ext = FT_ext_arr(:,k_ct);
            k_ct = k_ct + 1;
        else
            FT_ext = zeros(6,1);
        end

        flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext,'actual');
        
        k_act = k_act + 1;
    end
end

%% Plot the States and Animate
% state_plot(flight)
animation_plot(flight,wp,targ,'side');

% ukf_state_plot(sv, flight);
% state_plot(flight)
% fig_h_ani = animation_plot(flight, wp, camera);
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);
