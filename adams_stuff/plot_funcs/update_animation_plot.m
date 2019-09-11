function [h_persp, h_persp_est] = update_animation_plot(h_persp, traj_est_h, h_persp_est, pos, quat, sv, k, animation_pause)
    % update ground truth quad pose
    pos = pos(:);
    bRw = quat2rotm(quat(:)');

    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.1]';
    
    x_arrow = [pos pos + (bRw*vect_x)];
    y_arrow = [pos pos + (bRw*vect_y)];
    z_arrow = [pos pos + (bRw*vect_z)];
    h_persp = reassign(h_persp, x_arrow, y_arrow, z_arrow);

    % update estimate trajectory plot
    set(traj_est_h, 'xdata', sv.mu_hist(1, sv.hist_mask), ...
                    'ydata', sv.mu_hist(2, sv.hist_mask), ...
                    'zdata', sv.mu_hist(3, sv.hist_mask) );

    % update estimated quad pose
    quat_est = complete_unit_quat(sv.mu_hist(7:9, k));
    bRw_est = quat2rotm(quat_est');
    pos_est = sv.mu_hist(1:3, k);

    vect_x_est = [0.2 0.0 0.0]';
    vect_y_est = [0.0 0.2 0.0]';
    vect_z_est = [0.0 0.0 0.1]';
    
    x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
    y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
    z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];
    h_persp_est = reassign(h_persp_est, x_arrow_est, y_arrow_est, z_arrow_est);
    pause(animation_pause)
end