function [wp,wts,nom,model] = warm_start(wp,wts,model)

% Unpack stuff
N_wp   = size(wp.x,2);

% Initialize iLQR variables
nom.u_bar = [model.hover_u ; model.dt_ctl_max] .* ones(5,model.N_ctl);
nom.x_bar = zeros(13,model.N_ctl+1); 
nom.x_bar(:,1) = wp.x(:,1);

nom.l = zeros(5,1,model.N_ctl);
nom.L = zeros(5,13,model.N_ctl);

% Close-Enough Trajectory Plan
x_curr = wp.x(:,1);
for k = 2:N_wp
    x_wp = wp.x(:,k);
    
    nom = ilqr_x_sp(x_curr,x_wp,nom,wts,wp,model);
    fast_animation_plot(nom.x_bar,wp,'persp')
    
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
