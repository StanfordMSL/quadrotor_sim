function plot_bounding_angled_box(center_rc, width_pixel, height_pixel, box_ang, bb_rc_list, pos_w, quat, camera)
    % red lines are front, black are back, cyan are side. Solid lines are
    % top (top vertices have square markers)
    figure(3433); clf; axis equal;  hold on;
    plot(center_rc(2), center_rc(1), 'b*');
    rot_mat = [ cos(box_ang), -sin(box_ang);
                sin(box_ang),  cos(box_ang) ];
    bb_box = [center_rc(2), center_rc(1)] + ...
       ( rot_mat' * ...  % NOTE: transpose here is because we are flipping the y axis
        [-width_pixel/2,   -height_pixel/2;
          width_pixel/2, -height_pixel/2;
          width_pixel/2,  height_pixel/2;
         -width_pixel/2,  height_pixel/2;
         -width_pixel/2, -height_pixel/2]')';
    plot(bb_box(:,1), bb_box(:,2), 'b-')
    xlim([0, 2*camera.K(1,3)]); ylim([0, 2*camera.K(2,3)]); set(gca,'Ydir','reverse')
    [roll, pitch, yaw] = quat2angle(quat(:)', 'XYZ');
    text(10, 10, sprintf('pos (world frame): %.2f, %.2f, %.2f\nyaw, pitch, roll = %.1f, %.1f, %.1f\n', ...
        pos_w(1), pos_w(2), pos_w(2), ...
        yaw*180/pi, pitch*180/pi, roll*180/pi), 'VerticalAlignment','top');
    
    
    
    plot(bb_rc_list([1, 2], 2), bb_rc_list([1, 2], 1), 'rs', 'MarkerSize', 10) % mark the "front top" of the quad
    plot(bb_rc_list([3, 4], 2), bb_rc_list([3, 4], 1), 'ks', 'MarkerSize', 10) % mark the "back top" of the quad
    
    % front of the quad
    plot(bb_rc_list([1, 2], 2), bb_rc_list([1, 2], 1), 'r-', 'LineWidth', 3)
    plot(bb_rc_list([2, 6], 2), bb_rc_list([2, 6], 1), 'r:', 'LineWidth', 3)
    plot(bb_rc_list([6, 5], 2), bb_rc_list([6, 5], 1), 'r:', 'LineWidth', 3)
    plot(bb_rc_list([5, 1], 2), bb_rc_list([5, 1], 1), 'r:', 'LineWidth', 3)
    
    % back of the quad
    plot(bb_rc_list([4, 3], 2), bb_rc_list([4, 3], 1), 'k-', 'LineWidth', 3)
    plot(bb_rc_list([3, 7], 2), bb_rc_list([3, 7], 1), 'k:', 'LineWidth', 3)
    plot(bb_rc_list([7, 8], 2), bb_rc_list([7, 8], 1), 'k:', 'LineWidth', 3)
    plot(bb_rc_list([8, 4], 2), bb_rc_list([8, 4], 1), 'k:', 'LineWidth', 3)
    
    % sides of the quad
    plot(bb_rc_list([1, 4], 2), bb_rc_list([1, 4], 1), 'c-', 'LineWidth', 3)
    plot(bb_rc_list([2, 3], 2), bb_rc_list([2, 3], 1), 'c-', 'LineWidth', 3)
    plot(bb_rc_list([5, 8], 2), bb_rc_list([5, 8], 1), 'c:', 'LineWidth', 3)
    plot(bb_rc_list([6, 7], 2), bb_rc_list([6, 7], 1), 'c:', 'LineWidth', 3)
    
    
    plot(bb_box(:,1), bb_box(:,2), 'b-', 'LineWidth', 1)


    xlim([0, 2*camera.K(1,3)]); ylim([0, 2*camera.K(2,3)]); set(gca,'Ydir','reverse')
    [roll, pitch, yaw] = quat2angle(quat(:)', 'XYZ');
    text(10, 10, sprintf('pos (world frame): %.2f, %.2f, %.2f\nyaw, pitch, roll = %.1f, %.1f, %.1f\n', ...
        pos_w(1), pos_w(2), pos_w(2), ...
        yaw*180/pi, pitch*180/pi, roll*180/pi), 'VerticalAlignment','top');
end

    
%     quad_aligned_bb = [l/2, w/2, h/2;... %    1 front, left, up (from quad's perspective)
%                        l/2, -w/2, h/2;... %   2 front, right, up
%                        -l/2, -w/2, h/2;... %  3 back, right, up
%                        -l/2, w/2, h/2; ... %  4 back, left, up
%                        l/2, w/2, -h/2;... %   5 front, left, down
%                        l/2, -w/2, -h/2;... %  6 front, right, down
%                        -l/2, -w/2, -h/2;... % 7 back, right, down
%                        -l/2, w/2, -h/2 ]; %   8 back, left, down