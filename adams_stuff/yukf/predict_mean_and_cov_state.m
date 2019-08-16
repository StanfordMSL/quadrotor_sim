function [mu_bar, sig_bar, Wprime] = predict_mean_and_cov_state(sps, yukf, cov_add_noise)
    % inverse of the unscented transform
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    
    % Calculate mean (take into account quats!)
    [mu_bar, ei_vec_set] = calc_mean_quat(sps, yukf);
    
    % Calc covar (take into account quats!) - note ei_vec_set was calculated 
    % for the mean, but also can be used for the covar
    [sig_bar, Wprime] = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, cov_add_noise);
end