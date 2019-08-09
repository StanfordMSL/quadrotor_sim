clear
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time and Simulation Rate
tf = 9;
fc_hz = 50;
act_hz = 1000;

sim_dt = 1/lcm(fc_hz,act_hz);
sim_N  = tf/sim_dt;
t_fc   = 0:1/fc_hz:tf;
t_act  = 0:1/act_hz:tf; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation
model  = model_init('simple vII',act_hz,fc_hz); % Initialize Physics Model
s_est  = se_init('mocap',fc_hz);                % Initialize State Estimator
fc     = fc_init(model,'ilqr');                 % Initialize Controller
wp     = wp_init('half-square',0,tf,'no plot');% Initialize timestamped keyframes
FT_ext = nudge_init(act_hz,tf,'off');           % Initialize External Forces
flight = flight_init(model,tf,wp);              % Initialize Flight Variables
% t_comp = calc_init();                         % Initialize Compute Time Variables

k_fc  = 1;          % Flight Controller Time Counter
k_act = 1;          % Actual Dynamics Time Counter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Cold Start the nominal trajectory for the iLQR
nom = ilqr_init(flight.t_act(:,1),flight.x_act(:,1),wp,model);

for k = 1:sim_N
    % State Estimation and Control
    if (abs(t_fc(k_fc) - (k-1)*sim_dt) <= 1e-3) && (k_fc <= tf*fc_hz)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Display Current Time and Target Waypoint
        curr_time = t_fc(k_fc);
        curr_wp   = zeros(3,1);
        disp(['[main]: Current Time: ',num2str(curr_time),' Current WP: ',mat2str(curr_wp)]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % State Estimation
        x_fc = flight.x_act(:,k_act);
        
        % Control
        [curr_m_cmd, nom] = ilqr(curr_time,x_fc,nom,wp,fc,model);
        
        % Log State Estimation and Control
        flight.x_fc(:,k_fc)  = x_fc;
        flight.m_cmd(:,k_fc) = curr_m_cmd;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update Flight Controller Time Counter
        k_fc = k_fc + 1;
        
    end
    
    if abs(t_act(k_act) - (k-1)*sim_dt) <= 1e-3
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % Dynamic Model
        flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext(:,k_act),'actual');
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update Actual Dynamics Time Counter
        k_act = k_act + 1;
    end
end

%% Plot the States and Animate
state_plot(flight)
animation_plot(flight,wp)
% presentation_plot(time,x_act,quat,mu_ekf,mu_ukf);