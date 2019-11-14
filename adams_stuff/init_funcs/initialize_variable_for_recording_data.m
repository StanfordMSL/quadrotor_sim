function sv = initialize_variable_for_recording_data(x0_gt, x0_est, yukf, num_dims, num_timesteps)
    %%%  Init yukf save variable (sv) for later plotting %%%%%%%%%
    dim_covar = length(yukf.sigma);
    sv.mu_hist = zeros(num_dims, num_timesteps); 
    sv.mu_hist(:, 1) = x0_est;
    sv.dyn_ol_hist = sv.mu_hist; sv.do_ind = 2;% open loop propagation of dynamics
    sv.dyn_ol_hist2 = sv.dyn_ol_hist; % open loop propagation of dynamics
    sv.mu_act = zeros(num_dims, num_timesteps); sv.mu_act(:, 1) = x0_gt(:);
    sv.sig_hist = zeros(dim_covar, dim_covar, num_timesteps); sv.sig_hist(:, :, 1) = yukf.sigma;
    sv.sig_trace_hist = zeros(num_timesteps, 1); sv.sig_trace_hist(1) = trace(yukf.sigma);
    sv.time_hist = zeros(num_timesteps, 1); sv.time_hist(1) = 0;
    sv.hist_mask = false(num_timesteps, 1); sv.hist_mask(1) = true;

    qm = yukf.mu(7:10, 1);
    [roll, pitch, yaw] = quat2angle(qm(:)', 'XYZ');
    sv.ypr_hist = zeros(3, num_timesteps); sv.ypr_hist(:, 1) = [yaw; pitch; roll]*180/pi;

    qa = x0_gt(7:10); 
    [roll, pitch, yaw] = quat2angle(qa(:)', 'XYZ');
    sv.ypr_act_hist = zeros(3, num_timesteps); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll]*180/pi;

    sv.ang_err = zeros(num_timesteps, 1); sv.ang_err(1) = quat_dist(qa, qm);
    sv.ang = zeros(num_timesteps, 1); sv.ang(1) = 0;
    sv.ang_act = zeros(num_timesteps, 1); sv.ang_act(1) = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end