function [sig_bar, Wprime] = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, cov_add_noise)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff (sec. 4.5)
    dim_cov = size(yukf.sigma, 1);
    num_sps = size(sps, 2);
    Wprime = zeros(dim_cov, num_sps);
    
    W = yukf.w0_c;
    sig_bar = zeros(dim_cov, dim_cov);
    for sp_ind = 1:num_sps
        Wprime(1:9, sp_ind) = sps(1:9, sp_ind) - mu_bar(1:9); % still need to calculate the quat part of this
%         warning("check this next line!! may have been wrong before!")
        Wprime(10:12, sp_ind) = ei_vec_set(:, sp_ind);
        sig_bar = sig_bar + W * Wprime(:, sp_ind) * Wprime(:, sp_ind)';
        if(sp_ind == 1); W = yukf.wi; end % after first iter, weight changes to wi from w0
    end
    
    % lastly, add noise (need to confirm this is ok for quat parts too)    
    sig_bar = sig_bar + cov_add_noise;
end