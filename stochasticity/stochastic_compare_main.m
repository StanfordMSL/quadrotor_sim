clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Base Parameters
wp     = wp_init('climb');                         % Initialize mission
model  = model_init('v1.1.0','high-speed');    % Initialize quadcopter
wts    = wts_init();                                % Initialize State and Input Cost Weights
targ   = targ_init("none");                         % Initialize target

% iLQER Parameters
model.gamma = 1e-9;
W_leqr = diag(0.001*ones(13,1));
model.W_leqr_inv = inv(W_leqr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
cc_1 = 0;
cc_2 = 0;
cc_3 = 0;
% Warm Start with either differential flatness or iLQR itself
nom = df_init(wp,model,'yaw','hide');

n_exp = 100;
x_f_error = zeros(3,n_exp);
for k = 1:n_exp
    tic
    
    model.gamma = 1e-9;
    log1 = simulation(nom,wp,model,wts,targ,'ilqr','ideal');
    cc_1 = cc_1 + log1.cost_curr;
    x_f_error(1,k) = norm(log1.x_act(:,end) - wp.x(:,end));

    model.gamma = 1e-6;
    log2 = simulation(nom,wp,model,wts,targ,'ileqr','ideal');
    cc_2 = cc_2 +log2.cost_curr;
    x_f_error(2,k) = norm(log2.x_act(:,end) - wp.x(:,end));

%     model.gamma = 1e-6;
%     log3 = simulation(nom,wp,model,wts,targ,'ileqr_oa');
%     cc_3 = cc_3 + log3.cost_curr;

   disp(['Experiment #',num2str(k),' || Time Taken: ',num2str(toc)]);
end
cc_1 = cc_1./n_exp;
cc_2 = cc_2./n_exp;
cc_3 = cc_3./n_exp;

%% Plot the States and Animate
% figure(1)
% clf
% plot(cc_1,'--')
% hold on
% plot(cc_2,'--')
% plot(cc_3,'--');
% legend('cc_1','cc_2','cc_3');

% animation_plot_triple(log1,log2,log3,wp,targ,'side','show');
% animation_plot_dual(log1,log2,wp,targ,'side','show');

figure(3)
clf
plot(x_f_error(1,:)-x_f_error(2,:))
x_f_error_compare = [sum(x_f_error(1,:)) sum(x_f_error(2,:))] 