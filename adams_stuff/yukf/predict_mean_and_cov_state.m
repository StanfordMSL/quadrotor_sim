function [mu_bar, sig_bar, w0_c, wi] = predict_mean_and_cov_state(sps, yukf, cov_add_noise)
    % inverse of the unscented transform
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    
    dim = size(sps, 1);
    % calculate weights
    w0_m = yukf.prms.lambda / (yukf.prms.lambda + dim);
    w0_c = w0_m + 1 - yukf.prms.alpha^2 + yukf.prms.beta;
    wi = 1 / (2*(yukf.prms.lambda + dim));
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate mean
    mu_bar = (w0_m * sps(:,1)) + (wi*sum(sps(:,2:end), 2));
    % now overwrite the quaternion portion with the way it should have been updated...
    q_bar = calc_mean_quat(sps, w0_m, wi);
    mu_bar(7:9) = q_bar(2:4);
    
    
    sig_bar = w0_c*(sps(:,1)-mu_bar)*(sps(:,1)-mu_bar)';
    for k = 1:dim
        sig_bar = sig_bar + wi * (sps(:, k + 1) - mu_bar) * (sps(:, k+1) - mu_bar)';
        sig_bar = sig_bar + wi * (sps(:, dim + k + 1) - mu_bar)*(sps(:, dim + k + 1) - mu_bar)';
    end
    sig_bar = sig_bar + cov_add_noise;
end