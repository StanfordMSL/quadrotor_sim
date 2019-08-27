function output = get_aligned_bounding_box(bb_rc_list, x_curr, b_draw_box, camera)
    global t_tmp
    % unpack state
    pos_w = x_curr(1:3, 1);
    vel = x_curr(4:6, 1);
    quat = complete_unit_quat(x_curr(7:9, 1));
    wx = x_curr(10, 1);
    wy = x_curr(11, 1);
    wz = x_curr(12, 1);
    
    max_rc = max(bb_rc_list);
    min_rc = min(bb_rc_list);
    delta_rc = max_rc - min_rc;
    center_rc = (max_rc + min_rc)/2;

    width_pixel = delta_rc(2);
    height_pixel = delta_rc(1);

    if b_draw_box
        if mod(t_tmp, 0.5) <= 1/1000
            figure(3433); clf; axis equal;  hold on;
            plot_bounding_aligned_box(center_rc, delta_rc, bb_rc_list, pos_w, quat, camera);
            disp('')
        end
    end

    output = [center_rc(:); width_pixel; height_pixel];
end