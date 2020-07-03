clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('slit v0');                        % Initialize mission
model  = model_init('v1.0.1','high-speed');         % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("none");                         % Initialize target

% iLQER Parameters
model.gamma = 1e-6;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start

% Warm Start with either differential flatness or iLQR itself
% nom_base = df_init(wp,model,'yaw','hide');
nom_base = ilqr_init(wp,wts,model,'show');
nom1 = nom_base;
nom2 = nom_base;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

%% Test I
% ctrl_I = 't_ilqr';
% ctrl_II = 'ilqr';
% 
% model.alpha = 1.0;
% model.rho   = 1.0;
% log1 = simulation(nom1,wp,model,wts,targ,ctrl_I,'ideal');
% 
% model.alpha = 1.0;
% model.rho = 0.0;
% log2 = simulation(nom2,wp,model,wts,targ,ctrl_II,'ideal');

%% Test II
ctrl_I = 'ilqr';
ctrl_II = 'al_ilqr';

model.alpha = 1.0;
model.rho   = 1.0;
log1 = simulation(nom1,wp,model,wts,targ,ctrl_I,'ideal');

model.alpha = 0.1;
model.rho = 1.0;
log2 = simulation(nom2,wp,model,wts,targ,ctrl_II,'ideal');

%% Plot the States and Animate
animation_plot_dual(log1,log2,wp,targ,ctrl_I,ctrl_II,'persp','show');
% motor_plot_dual(log1,log2,model)