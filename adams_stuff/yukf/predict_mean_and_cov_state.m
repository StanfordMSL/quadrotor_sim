function [mu_bar, sig_bar] = predict_mean_and_cov_state(sps, yukf, cov_add_noise)
    global k
    % inverse of the unscented transform
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    
    % Calculate mean    
    mu_bar = (yukf.w0_m * sps(:, 1)) + (yukf.wi*sum(sps(:, 2:end), 2)); % calculate the mean of the vector parts
    [quat_mean, ei_vec_set] = calc_mean_quat(sps(7:10, :)', yukf); % now the quaternion parts
    [quat_mean_ego, ei_vec_set_ego] = calc_mean_quat(sps(20:23, :)', yukf); % now the quaternion parts
    [sps(20:23, :)'; quat_mean_ego]
    [sps(7:10, :)'; quat_mean]
    mu_bar(7:10) = quat_mean;
    mu_bar(20:23) = quat_mean_ego;
    
    % Calc covar (take into account quats!) - note ei_vec_set was calculated 
    % for the mean, but also can be used for the covar
    sig_bar = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, ei_vec_set_ego, cov_add_noise);
end