clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time and Simulation Rate
tf = 3;

lqr_hz = 0;         % iLQR Update Rate
ctl_hz = 200;       % Control Law Switch Rate
fbc_hz = 0;         % Feedback Controller Rate
est_hz = ctl_hz;    % State Estimator Sample Rate
act_hz = 1000;      % Actual Dynamics Update Rate

sim_dt = 1/act_hz;
sim_N  = tf/sim_dt;

t_est = 0:1/est_hz:tf;
t_lqr = 0:1/lqr_hz:tf;
t_ctl = 0:1/ctl_hz:tf;
t_act = 0:1/act_hz:tf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation

%%% Map, Dynamics and Control Initialization
model  = model_init('simple v0.0',est_hz,lqr_hz,ctl_hz,fbc_hz,act_hz); % Initialize Physics Model
wts    = wts_init();                            % Initialize Controller
wp     = wp_init('half-square',tf,'no plot');                       % Initialize timestamped keyframes
flight = logger_init(tf,wp,act_hz,ctl_hz);     % Initialize Flight Variables
targ   = targ_init("none");          % Initialize target

%%% Time Counters Initialization
k_est = 1;              % State Estimator Time Counter
k_lqr = 1;              % iLQR Update Time Counter
k_ctl = 1;              % Control Law Switch Time Counter
k_act = 1;              % Actual Dynamics Time Counter
k_wp  = 1;              % Waypoint Time Counter
tol   = 1e-1*sim_dt;    % Tolerance to trigger various processes

%%% Contact Parameter Initialization
k_ct  = 1;                   % Contact Force Time Counter
st_ct = 0;                   % Contact state (0 = no contact, 1 = contact/post-contact)
dt_ct = 0.2;                 % Duration of Contact Force
N_ct  = round(dt_ct*act_hz); % Number of actual dynamics frames

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Cold Start the nominal trajectory for the iLQR
nom = df_init(wp,model,'yaw');
nominal_plot(wp,nom,'persp',10);
disp('[main]: Diff. Flat. based warm start complete! Ready to launch!');
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

        k_est = k_est + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Open Loop Control Updater    
    if (abs(t_ctl(k_ctl)-sim_time) < tol) && (k_ctl <= tf*ctl_hz)
        [u,curr_m_cmd] = df_ol_con(k_ctl,nom,model,'ideal');
        
        % Log State Control Commands
        flight.m_cmd(:,k_ctl) = curr_m_cmd;  
        flight.u(:,k_ctl)     = u; 
        
        k_ctl = k_ctl + 1;
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
%state_plot(flight)
animation_plot(flight,wp,targ,'persp');
% motor_plot(flight,model);
