function [output, bb_rc_list] = predict_quad_bounding_box(x_curr, camera, initial_bb, yukf)
    % quad is z up, x forward (and y left)
    % camera is z out, x right and y down 
    % state is x_curr = [position - world; lin vel - world; quat [scal, x,
    %   y, z] - body frame; ang_vel - body frame] 
    %   note: R_w_quad = quat2rotm(quat(:)'); 
    
    
    global flight k_act
    
    output = [];
    b_draw_box = false; % setting to true slows it down considerably, but shows the prediction vs. true state & how the sigma points vary around the mean
    disp('')
    % unpack state
    pos_w = x_curr(1:3, 1);
    vel = x_curr(4:6, 1);
    quat = complete_unit_quat(x_curr(7:9, 1));
    wx = x_curr(10, 1);
    wy = x_curr(11, 1);
    wz = x_curr(12, 1);
    
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
    
    % construct sensor output
    if yukf.prms.b_angled_bounding_box
        output = get_angled_bounding_box(bb_rc_list, x_curr, b_draw_box, camera);
    else
        if yukf.prms.b_measure_everything
            output = x_curr;
        else
            output = get_aligned_bounding_box(bb_rc_list, x_curr, b_draw_box, camera);
        end
        
        quat_act = complete_unit_quat(flight.x_act(7:9, k_act));
        [yaw_act, pitch_act, roll_act] = quat2angle(quat_act(:)');
        pos_act = flight.x_act(1:3, k_act);
        
        if yukf.prms.b_measure_aspect_ratio
            output = [output; output(4)/output(3)];
        end
        if yukf.prms.b_measure_x
            output = [output; pos_act(1)];
        end
        if yukf.prms.b_measure_yaw
            output = [output; yaw_act];
        end
        if yukf.prms.b_measure_pitch
            output = [output; pitch_act];
        end
        if yukf.prms.b_measure_roll
            output = [output; roll_act];
        end
        if yukf.prms.b_measure_quat
            output = [output; quat_act(2:4)];
        end
        if yukf.prms.b_measure_pos
            output = [output; pos_act];
        end
%         output = output + (rand(size(output))-0.5)*3;
    end
        
end
