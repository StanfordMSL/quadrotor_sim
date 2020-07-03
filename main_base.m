clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('slit v0');                         % Initialize mission
model  = model_init('v1.0.0','high-speed');       % Initialize quadcopter
wts    = wts_init();                              % Initialize State and Input Cost Weights
targ   = targ_init("none");                       % Initialize target

% iLQER Parameters
model.gamma = 1e-6;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Warm Start with either differential flatness or iLQR itself
nom = df_init(wp,model,'yaw','hide');
% nom = ilqr_init(wp,wts,model,'hide');

% model.alpha = 0.11;
% model.rho = 1.0;
log = simulation(nom,wp,model,wts,targ,'t_ilqr','ideal');

%% Plot the States and Animate
motor_plot(log,model)
animation_plot(log,wp,targ,'persp','show');
