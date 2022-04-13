function [s_out,J_out,T_out,N_out] = param_est(Xact,Xdat,s0)

% Iteration Limit
Nlim = 300;

% Parameter Estimation
[s_comp,X_comp,T_comp,N_comp] = GN_comp(s0,Xdat,Nlim);                  % Gauss Newton (full kickback)
[s_indv,X_indv,T_indv,N_indv] = GN_indv(s0(3:4),Xdat,Nlim);   % Gauss Newton (standard)

% Performance Metrics
J_comp = J_calc(X_comp,Xact);
J_indv = J_calc(X_indv,Xact);

% Package for output
s_out = [s_comp(:,end) s_indv(:,end)];
J_out = [J_comp J_indv];
T_out = [T_comp T_indv];
N_out = [N_comp N_indv];

%% Debug Plotting
% figure(2)
% clf
% 
% subplot(2,1,1)
% plot(Xact(1,:),'LineWidth',2.0);
% hold on
% plot(Xdat(1,:),'--');
% plot(Xhat1(1,:),'LineWidth',2.0);
% plot(Xhat2(1,:),'LineWidth',2.0);
% legend('actual','data','est_{indv}','est_{indv}');
% 
% subplot(2,1,2)
% plot(Xact(2,:),'LineWidth',2.0);
% hold on
% plot(Xdat(2,:),'--');
% plot(Xhat1(2,:),'LineWidth',2.0);
% plot(Xhat2(2,:),'LineWidth',2.0);
% legend('actual','data','est_{indv}','est_{indv}');


end