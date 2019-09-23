function sigma_xz = calc_cross_correlation(sps_updated, mu_bar, z_hat, pred_obs, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    %  note: using Wprime instead of (sp - mu) avoids quat issues
    dim = length(yukf.mu);
    num_sps = size(sps_updated, 2);
    
    quat_ave_inv = quatinv(axang_to_quat(mu_bar(7:9))');
    
    Wprime = zeros(dim, num_sps);
    for sp_ind = 1:num_sps
        Wprime(:, sp_ind) = sps_updated(:, sp_ind) - mu_bar; % still need to overwrite the quat parts of this
        q1 = axang_to_quat(sps_updated(7:9, sp_ind));
        q_diff = quatmultiply(q1(:)', quat_ave_inv);
        axang_diff = quat_to_axang(q_diff);
        Wprime(7:9, sp_ind) = axang_diff(:); % overwrite quat parts here
    end

    sigma_xz = yukf.w0_c * Wprime(:, 1) * (pred_obs(:, 1) - z_hat)';
    for i = 2:num_sps
        sigma_xz = sigma_xz + yukf.wi * Wprime(:, i) * (pred_obs(:, i) - z_hat)';
    end
end