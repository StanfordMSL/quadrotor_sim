function [z_hat, S, S_inv] = predict_mean_and_cov_obs(pred_obs, yukf, cov_add_noise)
    % inverse of the unscented transform
    dim = size(pred_obs, 1);
    
    % Calculate mean
    z_hat = (yukf.w0_m * pred_obs(:, 1)) + (yukf.wi * sum(pred_obs(:, 2:end), 2));    
    
    % Calculate covar
    S = yukf.w0_c * (pred_obs(:, 1 )- z_hat) * (pred_obs(:, 1) - z_hat)';
    for obs_ind = 2:dim
        S = S + yukf.wi * (pred_obs(:, obs_ind + 1) - z_hat) * (pred_obs(:, obs_ind+1) - z_hat)';
        S = S + yukf.wi * (pred_obs(:, dim + obs_ind + 1) - z_hat) * (pred_obs(:, dim + obs_ind + 1) - z_hat)';
    end
    S = S + cov_add_noise;
    S_inv = inv(S);
end