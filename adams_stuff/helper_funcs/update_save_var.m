function sv = update_save_var(sv, save_index, yukf, flight, t_now)
    sv.mu_hist(:, save_index) = yukf.mu;
    sv.mu_act(:, save_index) = flight.x_act(:, save_index);

    sv.sig_hist(:, :, save_index) = yukf.sigma;
    sv.sig_trace_hist(save_index) = trace(yukf.sigma);
    sv.time_hist(save_index) = t_now;
    sv.hist_mask(save_index) = true;

    qm = yukf.mu(7:10); 
    [roll, pitch, yaw] = quat2angle(qm(:)', 'XYZ');
    sv.ypr_hist(:, save_index) = [yaw; pitch; roll]*180/pi;

    qa = flight.x_act(7:10, save_index); 
    [roll, pitch, yaw] = quat2angle(qa(:)', 'XYZ');
    sv.ypr_act_hist(:, save_index) = [yaw; pitch; roll]*180/pi;

    ang_err = quat_dist(qa, qm);
    q_mod = qa;
    if yukf.prms.b_enforce_yaw
        yaw = 0; % this is n
        q_mod = angle2quat(roll, pitch, 0, 'XYZ');
        ang_err = quat_dist(q_mod, qm);
    end
    sv.ang_err(save_index) = ang_err;
    sv.ang(save_index) = 2*acosd(qm(1));
    sv.ang_act(save_index) = 2*acosd(q_mod(1));
end