function [wp,wts,nom,model] = al_ilqr_init(wp,wts,model,nom_show)

% Unpack stuff
N_wp   = size(wp.x,2);

% Initialize iLQR variables
wrench = [model.m_est*model.g ; 0 ; 0 ; 0];
nom.u_bar = [wrench.*ones(4,model.N_ctl) ; model.dt_ctl_max.*ones(1,1000)];
nom.x_bar = zeros(14,model.N_ctl+1); 
nom.x_bar(7,:)  = 1;
nom.x_bar(14,:) = model.dt_ctl_max;
nom.x_bar(1:13,1) = wp.x(:,1);

nom.l = zeros(5,1,model.N_ctl);
nom.L = zeros(5,14,model.N_ctl);

tf = model.dt_ctl_max .* model.N_ctl; 
nom.t_bar = linspace(0,tf,model.N_ctl+1);

% Close-Enough Trajectory Plan
x_curr = wp.x(:,1);
for k = 2:N_wp
    x_wp = wp.x(:,k);
    
    nom = ilqr_x_sp(x_curr,x_wp,nom,wts,model);

    x_curr = nom.x_bar(:,end);
end

% % Publish some diagnostics
% switch nom_show
%     case 'show'
%         nominal_plot(wp,nom,'persp',10);
%     case 'hide'
% end
% disp('[ilqr_init]: Note, orientation data is lost in the wp2sigma step');
% disp(['[ilqr_init]: Trajectory has ',num2str(N_wp),' waypoints over ',num2str(tf),' seconds']);
% disp(['[ilqr_init]: Diff. Flat Trajectory Computed in: ',num2str(toc),' seconds.']);
% 
