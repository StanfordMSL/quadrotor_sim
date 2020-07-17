function [wp,wts,nom,model] = warm_start(wp,wts,model)

% Initialize iLQR variables
nom = toc_ilqr_init(601,16,wp,model);

% Warm start with an initial run
x_star = wp.x(:,2);
    
nom = toc_ilqr(x_star,nom,wts,wp,model);
% fast_animation_plot(nom.x_bar,wp,'persp')

% for k = 1:2
%     nom = toc_ilqr(x_star,nom,wts,wp,model);
%     fast_animation_plot(nom.x_bar,wp,'persp')
%     
%     n_inc = 50;
%     nom = toc_ilqr_upd(nom,n_inc,model);
%     total = nom.N_toc
% end

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
