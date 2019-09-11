function sv = update_save_var(sv, save_index, yukf, flight, t_now)
    sv.mu_hist(:, save_index) = yukf.mu;
    sv.mu_act(:, save_index) = flight.x_act(:, save_index);

    sv.sig_hist(:, :, save_index) = yukf.sigma;
    sv.sig_trace_hist(save_index) = trace(yukf.sigma);
    sv.time_hist(save_index) = t_now;
    sv.hist_mask(save_index) = true;

    qm = complete_unit_quat(yukf.mu(7:9)); 
    [yaw, pitch, roll] = quat2angle(qm(:)');
    sv.ypr_hist(:, save_index) = [yaw; pitch; roll]*180/pi;

    qa = complete_unit_quat(flight.x_act(7:9, save_index)); 
    [yaw, pitch, roll] = quat2angle(qa(:)');
    sv.ypr_act_hist(:, save_index) = [yaw; pitch; roll]*180/pi;

    sv.ang_err(save_index) = quat_dist(qa, qm);
    sv.ang(save_index) = 2*acosd(qm(1));
    sv.ang_act(save_index) = 2*acosd(qa(1));
end