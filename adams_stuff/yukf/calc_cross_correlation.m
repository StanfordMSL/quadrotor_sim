function sigma_xz = calc_cross_correlation(sps_updated, mu_bar, z_hat, pred_obs, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    %  note: using Wprime instead of (sp - mu) avoids quat issues
    dim_covar = length(yukf.sigma);
    num_sps = size(sps_updated, 2);
    
    quat_ave_inv1 = quatinv(mu_bar(7:10)');
    quat_ave_inv2 = quatinv(mu_bar(20:23)');
    
    Wprime = zeros(dim_covar, num_sps);
    for sp_ind = 1:num_sps
        Wprime(1:6, sp_ind) = sps_updated(1:6, sp_ind) - mu_bar(1:6);
        Wprime(10:12, sp_ind) = sps_updated(11:13, sp_ind) - mu_bar(11:13); % still need to overwrite the quat parts of this
        Wprime(13:18, sp_ind) = sps_updated(14:19, sp_ind) - mu_bar(14:19);
        Wprime(22:24, sp_ind) = sps_updated(24:26, sp_ind) - mu_bar(24:26); % still need to overwrite the quat parts of this
        
%         Wprime(:, sp_ind) = sps_updated(:, sp_ind) - mu_bar(:); % still need to overwrite the quat parts of this
        q1 = sps_updated(7:10, sp_ind);
        q_diff = quatmultiply(q1(:)', quat_ave_inv1);
        axang_diff = quat_to_axang(q_diff);
        Wprime(7:9, sp_ind) = axang_diff(:); % overwrite quat parts of tracked quad here
        
        q2 = sps_updated(20:23, sp_ind);
        q_diff2 = quatmultiply(q2(:)', quat_ave_inv2);
        axang_diff2 = quat_to_axang(q_diff2);
        Wprime(19:21, sp_ind) = axang_diff2(:); % overwrite quat parts of ego motion here
    end

    sigma_xz = yukf.w0_c * Wprime(:, 1) * (pred_obs(:, 1) - z_hat)';
    for i = 2:num_sps
        sigma_xz = sigma_xz + yukf.wi * Wprime(:, i) * (pred_obs(:, i) - z_hat)';
    end
end