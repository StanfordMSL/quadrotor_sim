function plot_3D_bb_world(initial_bb, tf_w_quad, pos_w)
    % red lines are front, black are back, cyan are side. Solid lines are
    % top (top vertices have square markers)
    ax_len = 0.06;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num_verts = size(initial_bb, 1);
    bb_w = (tf_w_quad * [initial_bb, ones(num_verts, 1)]')';
    bb_w = bb_w(:, 1:3);
    
    figure(9098); clf; hold on; axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z')
    plot3(pos_w(1), pos_w(2), pos_w(3), 'b*', 'MarkerSize', 4)
    plot3(bb_w([1, 2], 1), bb_w([1, 2], 2), bb_w([1, 2], 3), 'rs', 'MarkerSize', 10)
    plot3(bb_w([3, 4], 1), bb_w([3, 4], 2), bb_w([3, 4], 3), 'ks', 'MarkerSize', 10)
    
    plot3(bb_w([1, 2], 1), bb_w([1, 2], 2), bb_w([1, 2], 3), 'r-', 'LineWidth', 3)
    plot3(bb_w([2, 6], 1), bb_w([2, 6], 2), bb_w([2, 6], 3), 'r:', 'LineWidth', 3)
    plot3(bb_w([6, 5], 1), bb_w([6, 5], 2), bb_w([6, 5], 3), 'r:', 'LineWidth', 3)
    plot3(bb_w([5, 1], 1), bb_w([5, 1], 2), bb_w([5, 1], 3), 'r:', 'LineWidth', 3)
    
    plot3(bb_w([4, 3], 1), bb_w([4, 3], 2), bb_w([4, 3], 3), 'k-', 'LineWidth', 3)
    plot3(bb_w([3, 7], 1), bb_w([3, 7], 2), bb_w([3, 7], 3), 'k:', 'LineWidth', 3)
    plot3(bb_w([7, 8], 1), bb_w([7, 8], 2), bb_w([7, 8], 3), 'k:', 'LineWidth', 3)
    plot3(bb_w([8, 4], 1), bb_w([8, 4], 2), bb_w([8, 4], 3), 'k:', 'LineWidth', 3)
    
    plot3(bb_w([1, 4], 1), bb_w([1, 4], 2), bb_w([1, 4], 3), 'c-', 'LineWidth', 3)
    plot3(bb_w([2, 3], 1), bb_w([2, 3], 2), bb_w([2, 3], 3), 'c-', 'LineWidth', 3)
    plot3(bb_w([5, 8], 1), bb_w([5, 8], 2), bb_w([5, 8], 3), 'c:', 'LineWidth', 3)
    plot3(bb_w([6, 7], 1), bb_w([6, 7], 2), bb_w([6, 7], 3), 'c:', 'LineWidth', 3)
    
    % Determine World Frame Pose of Craft Axes
    R_w_quad = tf_w_quad(1:3, 1:3);
    x_arrow = [pos_w, pos_w + (R_w_quad * [ax_len; 0; 0])];
    y_arrow = [pos_w, pos_w + (R_w_quad * [0; ax_len; 0])];
    z_arrow = [pos_w, pos_w + (R_w_quad * [0; 0; ax_len])];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
    % Plot It!
    quad_ax_plot = plot3(x, y, z, 'linewidth',3);
    quad_ax_plot(1).Color = [1 0 0];
    quad_ax_plot(2).Color = [0 1 0];
    quad_ax_plot(3).Color = [0 0 1];
    
    
    
%     view([0, 90]) % [DEFAULT VIEW] looking in -Z direction (quad's top when at origin)
%     view([0, 0]) % looking in +Y direction (quad's right side when at origin)
    view([-90, 0]) % looking in +X direction (quad's back side when at origin)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%     quad_aligned_bb = [l/2, w/2, h/2;... %    1 front, left, up (from quad's perspective)
%                        l/2, -w/2, h/2;... %   2 front, right, up
%                        -l/2, -w/2, h/2;... %  3 back, right, up
%                        -l/2, w/2, h/2; ... %  4 back, left, up
%                        l/2, w/2, -h/2;... %   5 front, left, down
%                        l/2, -w/2, -h/2;... %  6 front, right, down
%                        -l/2, -w/2, -h/2;... % 7 back, right, down
%                        -l/2, w/2, -h/2 ]; %   8 back, left, down