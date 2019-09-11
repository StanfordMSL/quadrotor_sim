function plot_bounding_aligned_box(center_rc, delta_rc, bb_rc_list, pos_w, quat, camera)
    figure(3433); clf; axis equal;  hold on;
    plot(center_rc(2), center_rc(1), 'b*');

%     % plot full quad bb projection
%     quad_box = ...
%         [bb_rc_list(1, :); bb_rc_list(2,:); NaN, NaN;
%          bb_rc_list(2, :); bb_rc_list(6,:); NaN, NaN;
%          bb_rc_list(6, :); bb_rc_list(5,:); NaN, NaN;
%          bb_rc_list(5, :); bb_rc_list(1,:); NaN, NaN;
%          bb_rc_list(4, :); bb_rc_list(3,:); NaN, NaN;
%          bb_rc_list(3, :); bb_rc_list(7,:); NaN, NaN;
%          bb_rc_list(7, :); bb_rc_list(8,:); NaN, NaN;
%          bb_rc_list(8, :); bb_rc_list(4,:); NaN, NaN;
%          bb_rc_list(1, :); bb_rc_list(4,:); NaN, NaN;
%          bb_rc_list(2, :); bb_rc_list(3,:); NaN, NaN;
%          bb_rc_list(5, :); bb_rc_list(8,:); NaN, NaN;
%          bb_rc_list(6, :); bb_rc_list(7,:)];
%     plot(quad_box(:,2), quad_box(:,1), 'r-', 'LineWidth', 1)
    
    
    plot(bb_rc_list([1, 2], 2), bb_rc_list([1, 2], 1), 'rs', 'MarkerSize', 10) % mark the "front top" of the quad
    plot(bb_rc_list([3, 4], 2), bb_rc_list([3, 4], 1), 'bs', 'MarkerSize', 10) % mark the "back top" of the quad
    
    % front of the quad
    plot(bb_rc_list([1, 2], 2), bb_rc_list([1, 2], 1), 'r-', 'LineWidth', 3)
    plot(bb_rc_list([2, 6], 2), bb_rc_list([2, 6], 1), 'b-', 'LineWidth', 3)
    plot(bb_rc_list([6, 5], 2), bb_rc_list([6, 5], 1), 'g-', 'LineWidth', 3)
    plot(bb_rc_list([5, 1], 2), bb_rc_list([5, 1], 1), 'k-', 'LineWidth', 3)
    
    % back of the quad
    plot(bb_rc_list([4, 3], 2), bb_rc_list([4, 3], 1), 'r:', 'LineWidth', 3)
    plot(bb_rc_list([3, 7], 2), bb_rc_list([3, 7], 1), 'b:', 'LineWidth', 3)
    plot(bb_rc_list([7, 8], 2), bb_rc_list([7, 8], 1), 'g:', 'LineWidth', 3)
    plot(bb_rc_list([8, 4], 2), bb_rc_list([8, 4], 1), 'k:', 'LineWidth', 3)
    
    % sides of the quad
    plot(bb_rc_list([1, 4], 2), bb_rc_list([1, 4], 1), 'r--', 'LineWidth', 3)
    plot(bb_rc_list([2, 3], 2), bb_rc_list([2, 3], 1), 'b--', 'LineWidth', 3)
    plot(bb_rc_list([5, 8], 2), bb_rc_list([5, 8], 1), 'k--', 'LineWidth', 3)
    plot(bb_rc_list([6, 7], 2), bb_rc_list([6, 7], 1), 'g--', 'LineWidth', 3)
    
    
% 
    % plot output bounding box
    bb_box = [center_rc(2), center_rc(1)] + ...
        [-delta_rc(2)/2, -delta_rc(1)/2;
          delta_rc(2)/2, -delta_rc(1)/2;
          delta_rc(2)/2,  delta_rc(1)/2;
         -delta_rc(2)/2,  delta_rc(1)/2;
         -delta_rc(2)/2, -delta_rc(1)/2];
    plot(bb_box(:,1), bb_box(:,2), 'k-', 'LineWidth', 1)


    xlim([0, 2*camera.K(1,3)]); ylim([0, 2*camera.K(2,3)]); set(gca,'Ydir','reverse')
    [yaw, pitch, roll] = quat2angle(quat(:)');
    text(10, 10, sprintf('pos (world frame): %.2f, %.2f, %.2f\nyaw, pitch, roll = %.1f, %.1f, %.1f\n', ...
        pos_w(1), pos_w(2), pos_w(2), ...
        yaw*180/pi, pitch*180/pi, roll*180/pi), 'VerticalAlignment','top');
end