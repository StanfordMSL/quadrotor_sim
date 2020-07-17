clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('perch gentle');                         % Initialize mission
model  = model_init('v1.1.0','high-speed');    % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("branch");                         % Initialize target

% iLQER Parameters
model.gamma = 1e-6;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

% Warm Start with either differential flatness or iLQR itself
nom = df_init(wp,model,'yaw','hide');
% nom = ilqr_init(wp,wts,model);

log = simulation(nom,wp,model,wts,targ,'ilqr','ideal');

%% Plot the States and Animate
animation_plot(log,wp,targ,'persp','show');

%% Determine Trajectory Success/Failure
err_x_N = log.x_act(:,end) - wp.x(:,end);
err_Q = diag([ones(1,6) 0.1*ones(1,4) ones(1,3)]);
err_cost = err_x_N'*err_Q*err_x_N;
% err_tol = 0.2;    % Ballistic
err_tol = 0.05;     % Gentle

if err_cost < err_tol
    disp('[perch_sim]: Contact success');
else
    disp('[perch_sim]: Contact failed');
end
