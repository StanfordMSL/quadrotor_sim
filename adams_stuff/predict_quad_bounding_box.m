function  output = predict_quad_bounding_box(x_curr, camera, initial_bb)
    % quad is z up, x forward (and y left)
    % camera is z out, x right and y down 
    % state is x_curr = [position - world; lin vel - world; quat [scal, x,
    %   y, z] - body frame; ang_vel - body frame] 
    %   note: R_w_quad = quat2rotm(quat(:)'); 
    
    output = [];
    b_draw_box = false;
    b_angled_bounding_box = false;
    
    disp('')
    % unpack state
    pos_w = x_curr(1:3, 1);
    vel = x_curr(4:6, 1);
    q = x_curr(7:9, 1);
    q0 = sqrt(1 - q'*q);
    quat = [q0; q];
    wx = x_curr(10, 1);
    wy = x_curr(11, 1);
    wz = x_curr(12, 1);
    w_all = x_curr(10:12, 1);
    
    R_w_quad = quat2rotm(quat(:)'); % I confirmed this
    tf_w_quad = [R_w_quad, pos_w(:); [zeros(1, 3), 1]];
    
    tf_cam_quad = camera.tf_cam_w * tf_w_quad;
    
    num_verts = size(initial_bb, 1);
    bb_cam = (tf_cam_quad * [initial_bb, ones(num_verts, 1)]')';
    bb_cam = bb_cam(:, 1:3);
    
    % project each vertex into the pixel coordinates (row, col)
    %   save 3 corners
    bb_rc_list = zeros(num_verts, 2);
    for vert_ind = 1:num_verts
        bb_rc_list(vert_ind, :) = xyz_to_rc(bb_cam(vert_ind, :), camera);
    end
    
    if ~b_angled_bounding_box
        max_coords = max(bb_rc_list);
        min_coords = min(bb_rc_list);
        delta_rc = max_coords - min_coords;
        center_rc = (max_coords + min_coords)/2;
        
        b_use_qs_method = false;
        if b_use_qs_method
            pos_c = camera.tf_cam_w * [pos_w(:); 1];
            est_dist = norm(pos_c(1:3));
            width = initial_bb(1,2) * 2;
            height = initial_bb(1,3) * 2;
            width_pixel = camera.K_3x3(1, 1) * width / est_dist;
            height_pixel = camera.K_3x3(2, 2) * height / est_dist;
        else
            width_pixel = delta_rc(2);
            height_pixel = delta_rc(1);
        end
        
%         output = [center_rc(:); width_pixel; height_pixel; height_pixel/width_pixel];
        output = [center_rc(:); width_pixel; height_pixel];
        if b_draw_box
            figure(3433); clf; axis equal;  hold on;
            plot(center_rc(2), center_rc(1), 'b*');
            box = [center_rc(2), center_rc(1)] + ...
                [-delta_rc(2)/2, -delta_rc(1)/2;
                  delta_rc(2)/2, -delta_rc(1)/2;
                  delta_rc(2)/2,  delta_rc(1)/2;
                 -delta_rc(2)/2,  delta_rc(1)/2;
                 -delta_rc(2)/2, -delta_rc(1)/2];
            plot(box(:,1), box(:,2), 'b-')
            xlim([0, 2*camera.K_3x3(1,3)]); ylim([0, 2*camera.K_3x3(2,3)]); set(gca,'Ydir','reverse')
            [yaw, pitch, roll] = quat2angle(quat(:)');
            text(10, 10, sprintf('pos (world frame): %.2f, %.2f, %.2f\nyaw, pitch, roll = %.1f, %.1f, %.1f\n', ...
                pos_w(1), pos_w(2), pos_w(2), ...
                yaw*180/pi, pitch*180/pi, roll*180/pi), 'VerticalAlignment','top');
        end
        disp('')
    else
        % project center of quad into pixel coordinates (row, col)
        quad_center_xyz = tf_cam_quad(1:3, 4);
        quad_center_rc = xyz_to_rc(quad_center_xyz, camera);

        % calculate the angle of the bounding box - modify the angles to be 
        %   between +- 90 deg since the neural net wont be able to tell the 
        %   difference then keep the angle keeping the bounding box most horizontal

        flu = bb_rc_list(1, :); % front-left-up vextex
        fru = bb_rc_list(2, :); % front-right-up vextex
        blu = bb_rc_list(4, :); % back-left-up vextex
        angs = zeros(2, 1);
        angs(1) = wrapToPi(atan2(flu(2) - fru(2), flu(2) - flu(1))); 
        angs(2) = wrapToPi(atan2(blu(2) - flu(2), blu(2) - flu(1))); 
        signs = sign(angs);
        angs = abs(angs);
        angs(1) = min(angs(1), 180 - angs(1));
        angs(2) = min(angs(2), 180 - angs(2));
        [min_val, min_ind] = min(angs);
        box_ang = signs(min_ind) * min_val;
        
        % find the extent of the bounding box along the angled axes
        rot_mat = [ cos(box_ang), -sin(box_ang);
                    sin(box_ang),  cos(box_ang) ];
        pix_mean = mean(bb_rc_list);
        ax_aligned_pix = (rot_mat * (bb_rc_list - pix_mean)')';
        width_pixel = max(ax_aligned_pix(:, 2)) - min(ax_aligned_pix(:, 2));
        height_pixel = max(ax_aligned_pix(:, 1)) - min(ax_aligned_pix(:, 1));
        
        max_coords = max(bb_rc_list);
        min_coords = min(bb_rc_list);
        center_rc = (max_coords + min_coords)/2;
        
        output = [center_rc(:); width_pixel; height_pixel; box_ang];
        if b_draw_box
            figure(3433); clf; axis equal;  hold on;
            plot(center_rc(2), center_rc(1), 'b*');
            box = [center_rc(2), center_rc(1)] + ...
               ( rot_mat * ...
                [-width_pixel/2,   -height_pixel/2;
                  width_pixel/2, -height_pixel/2;
                  width_pixel/2,  height_pixel/2;
                 -width_pixel/2,  height_pixel/2;
                 -width_pixel/2, -height_pixel/2]')';
            plot(box(:,1), box(:,2), 'b-')
            xlim([0, 2*camera.K_3x3(1,3)]); ylim([0, 2*camera.K_3x3(2,3)]); set(gca,'Ydir','reverse')
            [yaw, pitch, roll] = quat2angle(quat(:)');
            text(10, 10, sprintf('pos (world frame): %.2f, %.2f, %.2f\nyaw, pitch, roll = %.1f, %.1f, %.1f\n', ...
                pos_w(1), pos_w(2), pos_w(2), ...
                yaw*180/pi, pitch*180/pi, roll*180/pi), 'VerticalAlignment','top');
        end
    end   
        
end
