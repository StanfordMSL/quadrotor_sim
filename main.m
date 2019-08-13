clear; clc; format compact
addpath(genpath(pwd));

tf = 10;
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
mu_curr = flight.x_act(:, 1);
dims = length(mu_curr);
sig_curr = eye(dims) * 0.01;
ukf_prms.alpha = 1; % scaling param - how far sig. points are from mean
ukf_prms.kappa = 2; % scaling param - how far sig. points are from mean
ukf_prms.beta = 2; % optimal choice according to prob rob
ukf_prms.lambda = ukf_prms.alpha^2*(dims + ukf_prms.kappa) - dims;
ukf_prms.meas_len = length(predict_quad_bounding_box(mu_curr, camera, initial_bb));
ukf_prms.R = eye(dims) * 0.01;
ukf_prms.Q = eye(ukf_prms.meas_len) * 0.01;
sv.mu_hist = zeros(dims, length(flight.t_act)); sv.mu_hist(:, 1) = mu_curr;
sv.sig_hist = zeros(dims, dims, length(flight.t_act)); sv.sig_hist(:, :, 1) = sig_curr;
sv.sig_trace_hist = zeros(length(flight.t_act), 1); sv.sig_trace_hist(1) = trace(sig_curr);
sv.time_hist = zeros(length(flight.t_act), 1); sv.time_hist(1) = 0;
sv.hist_mask = false(length(flight.t_act), 1); sv.hist_mask(1) = true;
q = mu_curr(7:9, 1);
quat = [sqrt(1 - q'*q); q];
[yaw, pitch, roll] = quat2angle(quat(:)');
sv.ypr_hist = zeros(3, length(flight.t_act)); sv.ypr_hist(:, 1) = [yaw; pitch; roll]*180/pi;
sv.ypr_act_hist = zeros(3, length(flight.t_act)); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll]*180/pi;

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
        
        yolo_output = predict_quad_bounding_box(flight.x_act(:,k_act), camera, initial_bb); % Add noise??
        [mu_out, sigma_out] = yukf_step(mu_curr, sig_curr, curr_m_cmd, yolo_output, model, camera, initial_bb, ukf_prms);
        sv.mu_hist(:, k_act) = mu_out;
        sv.sig_hist(:, :, k_act) = sigma_out;
        sv.sig_trace_hist(k_act) = trace(sigma_out);
        sv.time_hist(k_act) = curr_time;
        sv.hist_mask(k_act) = true;
        
        q = mu_out(7:9, 1); q = [sqrt(1 - q'*q); q];
        [yaw, pitch, roll] = quat2angle(q(:)');
        sv.ypr_hist(:, k_act) = [yaw; pitch; roll]*180/pi;
        
        q = flight.x_act(7:9,k_act); q = [sqrt(1 - q'*q); q];
        [yaw, pitch, roll] = quat2angle(q(:)');
        sv.ypr_act_hist(:, k_act) = [yaw; pitch; roll]*180/pi;
        disp('')
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Log the state est and motor cmd and update fc step tracker
        flight.x_fc(:,k_fc)  = x_est;
        flight.m_cmd(:,k_fc) = curr_m_cmd;

        if k_fc < length(flight.t_fc)
            k_fc = k_fc + 1;
        else
            % Reached the end. Don't update.
        end
    end
    
    % Actual Dynamics of Next Step
    flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext(:,k_act),'actual');
 
end

% % Plot the States and Animate
ukf_state_plot(sv, flight);

state_plot(flight)
fig_h_ani = animation_plot(flight, camera);
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);

% figure(3)
% plot(flight.t_log)
% ylim([0 3]);