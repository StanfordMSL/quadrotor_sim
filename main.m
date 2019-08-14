clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time and Simulation Rate
tf = 10;
<<<<<<< HEAD

est_hz = 200;       % State Estimator Time Counter
lqr_hz = 1;        % Low Rate Controller Sample Rate
con_hz = 50;       % High Rate Controller Sample Rate
act_hz = 1000;      % Actual Dynamics Sample Rate

sim_dt = 1/lcm(lcm(est_hz,con_hz),lcm(lqr_hz,act_hz));
sim_N  = tf/sim_dt;

t_est = 0:1/est_hz:tf;
t_lqr = 0:1/lqr_hz:tf;
t_con = 0:1/con_hz:tf;
t_act = 0:1/act_hz:tf; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation
model  = model_init('simple vII',est_hz,con_hz,act_hz); % Initialize Physics Model
s_est  = se_init('mocap',con_hz);                % Initialize State Estimator
fc     = fc_init(model,'ilqr');                 % Initialize Controller
wp     = wp_init('square',0,tf,'no plot');% Initialize timestamped keyframes
FT_ext = nudge_init(act_hz,tf,'off');           % Initialize External Forces
flight = flight_init(model,tf,wp);              % Initialize Flight Variables
% t_comp = calc_init();                         % Initialize Compute Time Variables

k_est = 1;          % State Estimator Time Counter
k_lqr = 1;          % Low Rate Controller Time Counter
k_con = 1;          % High Rate Controller Time Counter
k_act = 1;          % Actual Dynamics Time Counter
k_wp  = 1;          % Waypoint Time Counter

tol = 1e-5;         % Tolerance to trigger various processes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Cold Start the nominal trajectory for the iLQR
nom = ilqr_init(flight.t_act(:,1),flight.x_act(:,1),wp,fc,model);

for k = 1:sim_N
    sim_time = (k-1)*sim_dt;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if (abs(t_est(k_est)-sim_time) < tol) && (k_est <= tf*est_hz)
        % Perfect Sensing (used for flight control)
        x_fc = flight.x_act(:,k_act);
        flight.x_fc(:,k_est)  = x_fc;

        %%%%%%%%%%%%%%%%%%%%
        % Test UKF Goes Here
        
        %%%%%%%%%%%%%%%%%%%%
=======
fc_hz = 50;
act_hz = 1000;

model  = model_init('simple vII',act_hz,fc_hz); % Initialize Physics Model
s_est  = se_init('mocap',fc_hz);            % Initialize State Estimator
fc     = fc_init(model,'ilqr');             % Initialize Controller
wp     = wp_init('square',tf,'no plot');     % Initialize timestamped keyframes
FT_ext = nudge_init(act_hz,tf,'off');             % Initialize External Forces
flight = flight_init(model,tf,wp);             % Initialize Flight Variables

% Simulation
k_fc = 1;       % Flight Control Step Counter
k_kf = 1;       % Keyframe Step Counter

flight.t_log = 0;
initial_bb = init_quad_bounding_box();
camera = init_camera();

% INIT YUKF %%%%%%%%%%%%%%%%%%
mu_curr = flight.x_act(:, 1); mu_prev = mu_curr;
dims = length(mu_curr);
sig_curr = eye(dims) * 0.01;
ukf_prms.alpha = 1; % scaling param - how far sig. points are from mean
ukf_prms.kappa = 2; % scaling param - how far sig. points are from mean
ukf_prms.beta = 2; % optimal choice according to prob rob
ukf_prms.lambda = ukf_prms.alpha^2*(dims + ukf_prms.kappa) - dims;
ukf_prms.meas_len = length(predict_quad_bounding_box(mu_curr, camera, initial_bb));

ukf_prms.R = eye(dims) * 0.01;
ukf_prms.Q = eye(ukf_prms.meas_len) * .1;

% yukf save variable for later plotting
sv.mu_hist = zeros(dims, length(flight.t_act)); sv.mu_hist(:, 1) = mu_curr;
sv.sig_hist = zeros(dims, dims, length(flight.t_act)); sv.sig_hist(:, :, 1) = sig_curr;
sv.sig_trace_hist = zeros(length(flight.t_act), 1); sv.sig_trace_hist(1) = trace(sig_curr);
sv.time_hist = zeros(length(flight.t_act), 1); sv.time_hist(1) = 0;
sv.hist_mask = false(length(flight.t_act), 1); sv.hist_mask(1) = true;

qtmp = mu_curr(7:9, 1);
qm = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qm(:)');
sv.ypr_hist = zeros(3, length(flight.t_act)); sv.ypr_hist(:, 1) = [yaw; pitch; roll];

qtmp = flight.x_act(7:9 ,1); 
qa = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qa(:)');
sv.ypr_act_hist = zeros(3, length(flight.t_act)); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll];

sv.ang_err = zeros(length(flight.t_act), 1); sv.ang_err(1) = quat_dist(qa, qm);
sv.ang = zeros(length(flight.t_act), 1); sv.ang(1) = 0;
sv.ang_act = zeros(length(flight.t_act), 1); sv.ang_act(1) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k_act = 1:(length(flight.t_act)-1)
    curr_time = round(flight.t_act(k_act),2);
       
    % State Estimation and Control
    if abs(curr_time - flight.t_fc(k_fc)) < 1e-6
        % Display Current Time and Current Time
        curr_wp = wp.x(1:3,fc.wp);
        disp(['[main]: Current Time: ',num2str(curr_time),' Current WP: ',mat2str(curr_wp)]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % State Estimation
        x_est = flight.x_act(:,k_act);
        
        % Control
        [curr_m_cmd, fc] = ilqr(x_est,model,fc,wp);
        
        % Sensing
        yolo_output = predict_quad_bounding_box(flight.x_act(:,k_act), camera, initial_bb); % Add noise??
        
        % Tracking (UKF)
        u_est = [];
%         u_est = curr_m_cmd;
        [mu_out, sigma_out] = yukf_step(mu_curr, sig_curr, [], yolo_output, model, camera, initial_bb, ukf_prms, mu_prev);
        mu_prev = mu_curr;
%         mu_curr = mu_out;
%         sig_curr = sigma_out;
        
        % Save values for plotting %%%%%%%%%%%%%%%%%%
        sv.mu_hist(:, k_act) = mu_out;
        sv.sig_hist(:, :, k_act) = sigma_out;
        sv.sig_trace_hist(k_act) = trace(sigma_out);
        sv.time_hist(k_act) = curr_time;
        sv.hist_mask(k_act) = true;
        
        qtmp = mu_out(7:9, 1); 
        qm = [sqrt(1 - qtmp'*qtmp); qtmp];
        [yaw, pitch, roll] = quat2angle(qm(:)');
        sv.ypr_hist(:, k_act) = [yaw; pitch; roll];
        
        qtmp = flight.x_act(7:9,k_act); 
        qa = [sqrt(1 - qtmp'*qtmp); qtmp];
        [yaw, pitch, roll] = quat2angle(qa(:)');
        sv.ypr_act_hist(:, k_act) = [yaw; pitch; roll];
        
        sv.ang_err(k_act) = quat_dist(qa, qm);
        sv.ang(k_act) = 2*acos(qm(1));
        sv.ang_act(k_act) = 2*acos(qa(1));
        disp('')
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
>>>>>>> 02592ca4c06703cc5d97dcfbe3a385f327051f3b
        
        k_est = k_est + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Rate Controller    
    if (abs(t_lqr(k_lqr)-sim_time) < tol) && (k_lqr <= tf*lqr_hz)
        % Display Current Time and Target Waypoint
        t_now = t_con(k_con);
        
        if t_now >= wp.t(k_wp)
            k_wp = k_wp + 1;
        end
        
        curr_wp   = wp.x(k_wp);
%         disp(['[main]: L-R Controller Current Time: ',num2str(t_now),' Current WP: ',mat2str(curr_wp)]);

        % Update LQR params
%         nom = ilqr(t_now,x_fc,wp,nom,fc,model);
        k_lqr = k_lqr + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Rate Controller    
    if (abs(t_con(k_con)-sim_time) < tol) && (k_con <= tf*con_hz)
        % Draw Out Motor Commands from u_bar computed by iLQR
        del_x = x_fc-nom.x_bar(:,k_con);
        u  = nom.u_bar(:,k_con) + nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
        curr_m_cmd = direct_motor_control(u,model);
%         curr_m_cmd = direct_motor_control(nom.u_bar(:,k_con),model);

<<<<<<< HEAD
        % Log State Estimation and Control
        flight.m_cmd(:,k_con) = curr_m_cmd;       
        
        k_con = k_con + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    if (abs(t_act(k_act)-sim_time) < tol)
        flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext(:,k_act),'actual');
        
        k_act = k_act + 1;
    end
end
=======
% % Plot the States and Animate
ukf_state_plot(sv, flight);

state_plot(flight)
fig_h_ani = animation_plot(flight, wp, camera);
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);
>>>>>>> 02592ca4c06703cc5d97dcfbe3a385f327051f3b

%% Plot the States and Animate
% state_plot(flight)
animation_plot(flight,wp)
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);