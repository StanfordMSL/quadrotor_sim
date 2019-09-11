function [sv, yukf] = yolo_ukf(yukf, sv, flight, k_act, t_now, initial_bb, camera, model, u_est)
    global t_tmp
    
    % fake sensor measurement using ground truth state and "perfect" bounding box placement
    yolo_output = predict_quad_bounding_box(flight.x_act(:, k_act), camera, initial_bb, yukf); % Add noise??
    
    if false && mod(t_tmp, 1.0) <= 1/1000
        test_bounding_box_code(flight.x_act(:, k_act))
        meas_str = repmat('%.4f, ', 1, yukf.prms.meas_len);
        fprintf(['t_now = %.3f s, z_t = [', meas_str(1:end-2), ']\n'], t_tmp, yolo_output)
        fprintf('top_left_cr: (%.1f, %.1f), bottom_right_cr: (%.1f, %.1f)\n', yolo_output(2) - yolo_output(3)/2, yolo_output(1) - yolo_output(4)/2, yolo_output(2) + yolo_output(3)/2, yolo_output(1) + yolo_output(4)/2)
        disp('')
    end

    yukf = yukf_step(yukf, u_est, yolo_output, model, camera, initial_bb);
    if(~isempty(u_est))
        dyn_only = propagate_state(sv.dyn_ol_hist(:, sv.do_ind-1), model, u_est, yukf.dt);
        sv.dyn_ol_hist(:, sv.do_ind) = dyn_only; 
        sv.do_ind = sv.do_ind + 1;
    end
    
    % Save values for plotting %%%%%%%%%%%%%%%%%%
    sv = update_save_var(sv, k_act, yukf, flight, t_now);
    disp('')
end