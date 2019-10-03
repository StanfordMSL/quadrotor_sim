function sigma_xz = calc_cross_correlation(sps_updated, mu_bar, z_hat, pred_obs, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    %  note: using Wprime instead of (sp - mu) avoids quat issues
    dim_covar = length(yukf.sigma);
    num_sps = size(sps_updated, 2);
    
    quat_ave_inv = quatinv(axang_to_quat(mu_bar(7:9))');
    
    Wprime = zeros(dim_covar, num_sps);
    for sp_ind = 1:num_sps
        Wprime(1:6, sp_ind) = sps_updated(1:6, sp_ind) - mu_bar(1:6);
        Wprime(10:12, sp_ind) = sps_updated(11:13, sp_ind) - mu_bar(11:13); % still need to overwrite the quat parts of this
        q1 = sps_updated(7:10, sp_ind);
        q_diff = quatmultiply(q1(:)', quat_ave_inv);
        axang_diff = quat_to_axang(q_diff);
        Wprime(7:9, sp_ind) = axang_diff(:); % overwrite quat parts here
    end

    sigma_xz = yukf.w0_c * Wprime(:, 1) * (pred_obs(:, 1) - z_hat)';
    for i = 2:num_sps
        sigma_xz = sigma_xz + yukf.wi * Wprime(:, i) * (pred_obs(:, i) - z_hat)';
    end
end