function [sv, yukf] = yolo_ukf(yukf, sv, flight, k_est, k_act, t_now, initial_bb, camera, model)
    % fake sensor measurement using ground truth state and "perfect"
    % bounding box placement
    x_act_permuted = [flight.x_act(1:6, k_act); flight.x_act(10:12, k_act); complete_unit_quat(flight.x_act(7:9, k_act))];
    yolo_output = predict_quad_bounding_box(x_act_permuted, camera, initial_bb); % Add noise??

    u_est = []; % u_est = curr_m_cmd;
    yukf = yukf_step(yukf, u_est, yolo_output, model, camera, initial_bb);

    % Save values for plotting %%%%%%%%%%%%%%%%%%
    sv.mu_hist(:, k_act) = yukf.mu;
    sv.mu_act(:, k_act) = x_act_permuted;
    
    sv.sig_hist(:, :, k_act) = yukf.sigma;
    sv.sig_trace_hist(k_act) = trace(yukf.sigma);
    sv.time_hist(k_act) = t_now;
    sv.hist_mask(k_act) = true;

    qm = yukf.mu(10:13); 
    [yaw, pitch, roll] = quat2angle(qm(:)');
    sv.ypr_hist(:, k_act) = [yaw; pitch; roll]*180/pi;

    qa = x_act_permuted(10:13); 
    [yaw, pitch, roll] = quat2angle(qa(:)');
    sv.ypr_act_hist(:, k_act) = [yaw; pitch; roll]*180/pi;

    sv.ang_err(k_act) = quat_dist(qa, qm);
    sv.ang(k_act) = 2*acosd(qm(1));
    sv.ang_act(k_act) = 2*acosd(qa(1));
    disp('')
end