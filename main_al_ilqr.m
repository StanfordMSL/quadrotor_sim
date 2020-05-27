clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('climb');                         % Initialize mission
model  = model_init('v1.0.0','high-speed');    % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("none");                         % Initialize target

% iLQER Parameters
model.gamma = 1e-6;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Warm Start with either differential flatness or iLQR itself
nom_base = df_init(wp,model,'yaw','hide');
% nom = ilqr_init(wp,wts,model);
nom1 = nom_base;
nom2 = nom_base;

log1 = simulation(nom1,wp,model,wts,targ,'ilqr','ideal');
log2 = simulation(nom2,wp,model,wts,targ,'al_ilqr','ideal');

%% Plot the States and Animate
animation_plot_dual(log1,log2,wp,targ,'persp','show');
motor_plot_dual(log1,log2,model)