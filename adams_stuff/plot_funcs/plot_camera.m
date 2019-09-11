function [lims, plt_h] = plot_camera(camera, fig_h)
    figure(fig_h);
    xyzs = (camera.tf_w_cam * [camera.viz_wf, ones(size(camera.viz_wf, 1), 1)]')';
    plt_h = plot3(xyzs(:, 1), xyzs(:, 2), xyzs(:,3), 'm-');
    
    % expand the plot to show the camera
    lims = [min(xyzs(:,1:3)); max(xyzs(:,1:3))];
    xlim( [ min([lims(:, 1)', xlim]), max([lims(:, 1)', xlim]) ] );
    ylim( [ min([lims(:, 2)', ylim]), max([lims(:, 2)', ylim]) ] );
    zlim( [ min([lims(:, 3)', zlim]), max([lims(:, 3)', zlim]) ] );
    
%     % show camera field of view 
%     full_range = ceil(xlim * [-1; 1] - camera.draw_len);
%     draw_period = 4;
%     K_inv = inv(camera.K);
%     for d = (camera.draw_len + draw_period) : draw_period : full_range
%        corners = d * K_inv * [0, 0, 1;
%                               0, 2*camera.K(2, 3), 1;
%                               2 * camera.K(1, 3), 2 * camera.K(2, 3), 1;
%                               2*camera.K(1, 3), 0, 1]';
%        corners = camera.tf_w_cam * [corners; ones(1, 4)];
%        corners = [corners(1:(end-1), :), corners(1:(end-1), 1)]';
%        plot3(corners(:, 1), corners(:, 2), corners(:, 3), 'b-');
%     end
end