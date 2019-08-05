clear
addpath(genpath(pwd));

tf = 10;
fc_hz = 100;
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
state_plot(flight)
animation_plot(flight)
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);

% figure(3)
% plot(flight.t_log)
% ylim([0 3]);