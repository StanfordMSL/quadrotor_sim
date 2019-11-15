function sig_bar = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, ei_vec_set_ego, cov_add_noise)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff (sec. 4.5)
    dim = length(yukf.sigma);
    num_sps = size(sps, 2);
    Wprime = zeros(dim, num_sps);
    
    W = yukf.w0_c;
    sig_bar = zeros(dim, dim);
    for sp_ind = 1:num_sps
        Wprime(1:6, sp_ind) = sps(1:6, sp_ind) - mu_bar(1:6); % still need to overwrite the quat parts of this
        Wprime(10:12, sp_ind) = sps(11:13, sp_ind) - mu_bar(11:13); % still need to overwrite the quat parts of this
        Wprime(7:9, sp_ind) = ei_vec_set(:, sp_ind);
        
        Wprime(13:18, sp_ind) = sps(14:19, sp_ind) - mu_bar(14:19);
        Wprime(22:24, sp_ind) = sps(24:26, sp_ind) - mu_bar(24:26); % still need to overwrite the quat parts of this
        Wprime(19:21, sp_ind) = ei_vec_set_ego(:, sp_ind);
        
        sig_bar = sig_bar + W * Wprime(:, sp_ind) * Wprime(:, sp_ind)';
        if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
    end
    
    % lastly, add noise (need to confirm this is ok for quat parts too)    
    sig_bar = sig_bar + cov_add_noise;
    
    % project sig_bar to pos. def. cone to avoid numeric issues
    sig_bar = enforce_pos_def_sym_mat(sig_bar);
end
