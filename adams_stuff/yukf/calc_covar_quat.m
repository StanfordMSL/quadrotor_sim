function sig_bar = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, cov_add_noise)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff (sec. 4.5)
    dim = size(sps, 1);
    num_sps = size(sps, 2);
    Wprime = zeros(dim, num_sps);
    
    W = yukf.w0_c;
    sig_bar = zeros(dim, dim);
    for sp_ind = 1:num_sps
        Wprime(:, sp_ind) = sps(:, sp_ind) - mu_bar; % still need to overwrite the quat parts of this
        Wprime(7:9, sp_ind) = ei_vec_set(:, sp_ind);
%         Wprime(7:9, sp_ind) = ei_quat_set(2:4, sp_ind);
        sig_bar = sig_bar + W * Wprime(:, sp_ind) * Wprime(:, sp_ind)';
        if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
    end
    
    % lastly, add noise (need to confirm this is ok for quat parts too)    
    sig_bar = sig_bar + cov_add_noise;
end