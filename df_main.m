clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time and Simulation Rate
tf = 1;

est_hz = 100;       % State Estimator Time Counter
lqr_hz = 2;         % Low Rate Controller Sample Rate
con_hz = 100;       % High Rate Controller Sample Rate
act_hz = 100;      % Actual Dynamics Sample Rate

sim_dt = 1/lcm(lcm(est_hz,con_hz),lcm(lqr_hz,act_hz));
sim_N  = tf/sim_dt;

t_est = 0:1/est_hz:tf;
t_lqr = 0:1/lqr_hz:tf;
t_con = 0:1/con_hz:tf;
t_act = 0:1/act_hz:tf; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation

%%% Map, Dynamics and Control Initialization
model  = model_init('simple vII',est_hz,lqr_hz,con_hz,act_hz); % Initialize Physics Model
fc     = fc_init(model,'ilqr');                         % Initialize Controller
wp     = wp_init('line',0,tf,'no plot');              % Initialize timestamped keyframes
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
nom = df_init(wp,model);
nominal_plot(wp,nom,'persp',10);
