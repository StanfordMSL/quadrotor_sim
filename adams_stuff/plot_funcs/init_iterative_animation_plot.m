function [traj_est_h, h_persp, h_persp_est] = init_iterative_animation_plot(init_pos, init_quat, camera, position_mat, quat_mat, b_view_from_camera_perspective)
    fig_ani = figure(324234); clf; grid on; hold on
        
    % plot limits
    extra = 3;
    xlim([min([position_mat(:, 1) - extra]),  max([position_mat(:, 1) + extra])]);
    ylim([min([position_mat(:, 2) - extra]),  max([position_mat(:, 2) + extra])]);
    zlim([min([position_mat(:, 3) - extra]),  max([position_mat(:, 3) + extra])]);

    % Plot the camera (and expand axis to show it)
    [~, cmr_h] = plot_camera(camera, fig_ani);
    if b_view_from_camera_perspective
        set(gca, 'CameraPosition', camera.tf_w_cam(1:3, 4)'); 
        view(-90,0); % look from behind the camera
    else
        view(320, 20);
    end

    % plot the ground truth trajectory & initialize the estimated trajectory
    traj_h = plot3(position_mat(:, 1), position_mat(:, 2), position_mat(:, 3), 'b-');
    traj_est_h = plot3(NaN, NaN, NaN, 'r-');

    %%% plot initial ground truth quad
    % Body Frame Axes
    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.1]';

    % Construct Rotation Matrix
    quat = quat_mat(1, :);
    bRw = quat2rotm(quat);

    % Determine World Frame Pose of Craft Axes
    pos = position_mat(1, :)';

    x_arrow = [pos pos + (bRw*vect_x)];
    y_arrow = [pos pos + (bRw*vect_y)];
    z_arrow = [pos pos + (bRw*vect_z)];

    x = [x_arrow(1,:); y_arrow(1,:); z_arrow(1,:)]';
    y = [x_arrow(2,:); y_arrow(2,:); z_arrow(2,:)]';
    z = [x_arrow(3,:); y_arrow(3,:); z_arrow(3,:)]';

    % Plot It!
    h_persp = plot3(x, y, z, 'linewidth', 3);

    % Set the Correct Colors
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];
    %%%%%%%%%%%

    %%% plot initial estimated quad
    % Body Frame Axes
    vect_x_est = [0.2 0.0 0.0]';
    vect_y_est = [0.0 0.2 0.0]';
    vect_z_est = [0.0 0.0 0.1]';

    % Construct Rotation Matrix
    quat_est = complete_unit_quat(init_quat(end-2:end));
    bRw_est = quat2rotm(quat_est(:)');

    % Determine World Frame Pose of Craft Axes
    pos_est = init_pos(:);

    x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
    y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
    z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];

    x_est = [x_arrow_est(1,:); y_arrow_est(1,:); z_arrow_est(1,:)]';
    y_est = [x_arrow_est(2,:); y_arrow_est(2,:); z_arrow_est(2,:)]';
    z_est = [x_arrow_est(3,:); y_arrow_est(3,:); z_arrow_est(3,:)]';

    % Plot It!
    h_persp_est = plot3(x_est, y_est, z_est, 'linewidth', 3);

    % Set the Correct Colors
    h_persp_est(1).Color = [0.8 0 0];
    h_persp_est(2).Color = [0 0.8 0];
    h_persp_est(3).Color = [0 0 0.8];
    %%%%%%%%%%%%%

    % set the labels & legend  
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis');
    legend([cmr_h; traj_h; traj_est_h; h_persp; h_persp_est], {'camera', 'traj._{gt}', 'traj._{est}', 'X_{gt}','Y_{gt}','Z_{gt}', 'X_{est}','Y_{est}','Z_{est}'});
    axis equal
end