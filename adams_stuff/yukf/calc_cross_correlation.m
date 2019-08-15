function sigma_xz = calc_cross_correlation(Wprime, z_hat, pred_obs, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    %  note: using Wprime instead of (sp - mu) avoids quat issues
    num_sp = size(Wprime, 2);
    sigma_xz = yukf.w0_c * Wprime(:, 1) * (pred_obs(:, 1) - z_hat)';
    for i = 2:num_sp
        sigma_xz = sigma_xz + yukf.wi * Wprime(:, i) * (pred_obs(:, i) - z_hat)';
    end
end