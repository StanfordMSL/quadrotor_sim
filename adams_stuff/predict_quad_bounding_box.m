function [output, bb_rc_list] = predict_quad_bounding_box(x_curr, camera, initial_bb, yukf)
    % basic output is [r_center, c_center, width, height, ...]
    % quad is z up, x forward (and y left)
    % camera is z out, x right and y down 
    % state is x_curr = [position - world; lin vel - world; quat [scal, x,
    %   y, z] - body frame; ang_vel - body frame] 
    %   note: R_w_quad = quat2rotm(quat(:)');     
    global flight k_act k
    
    b_draw_box = false; % setting to true slows it down considerably, but shows the prediction vs. true state & how the sigma points vary around the mean
    
    tf_w_quad = state_to_tf(x_curr);
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
    
    % construct sensor output
    if yukf.prms.b_angled_bounding_box
        output = get_angled_bounding_box(bb_rc_list, x_curr, b_draw_box, camera);
    else
        output = get_aligned_bounding_box(bb_rc_list, x_curr, b_draw_box, camera);
    end
    if isempty(k_act) && isempty(k)
        state_gt = flight.x_act(:, 1);
    elseif isempty(k)
        state_gt = flight.x_act(:, k_act);
    else
        state_gt = flight.x_act(:, k);
    end
    output = augment_measurement(output, yukf, x_curr, state_gt);
end
