clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('dive');                         % Initialize mission
model  = model_init('simple v0.8','high-speed');    % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("none");                         % Initialize target

% iLQER Parameters
model.gamma = 1e-6;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Warm Start with either differential flatness or iLQR itself
nom = df_init(wp,model,'yaw','hide');
% nom = ilqr_init(wp,wts,model);

log = simulation(nom,wp,model,wts,targ,'ilqr');

%% Plot the States and Animate
animation_plot(log,wp,targ,'persp','show');
