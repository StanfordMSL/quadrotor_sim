function [sv, yukf] = yolo_ukf(yukf, sv, flight, k_est, t_now, initial_bb, camera, model)
    % fake sensor measurement using ground truth state and "perfect"
    % bounding box placement
    yolo_output = predict_quad_bounding_box(flight.x_act(:, k_est), camera, initial_bb); % Add noise??

    u_est = []; % u_est = curr_m_cmd;
    yukf = yukf_step(yukf, u_est, yolo_output, model, camera, initial_bb);

    % Save values for plotting %%%%%%%%%%%%%%%%%%
    sv.mu_hist(:, k_est) = yukf.mu;
    sv.sig_hist(:, :, k_est) = yukf.sigma;
    sv.sig_trace_hist(k_est) = trace(yukf.sigma);
    sv.time_hist(k_est) = t_now;
    sv.hist_mask(k_est) = true;

    qm = complete_unit_quat(yukf.mu(7:9)); 
    [yaw, pitch, roll] = quat2angle(qm(:)');
    sv.ypr_hist(:, k_est) = [yaw; pitch; roll];

    qa = complete_unit_quat(flight.x_act(7:9, k_est)); 
    [yaw, pitch, roll] = quat2angle(qa(:)');
    sv.ypr_act_hist(:, k_est) = [yaw; pitch; roll];

    sv.ang_err(k_est) = quat_dist(qa, qm);
    sv.ang(k_est) = 2*acos(qm(1));
    sv.ang_act(k_est) = 2*acos(qa(1));
    disp('')
end