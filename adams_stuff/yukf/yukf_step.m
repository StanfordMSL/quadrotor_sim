function yukf = yukf_step(yukf, u, z, model, camera, initial_bb)
    dim_state = length(yukf.mu);
    num_sp = 2 * size(yukf.sigma, 1) + 1;
    
    % line 2 prob rob
    sps = calc_sigma_points(yukf.mu, yukf.sigma, yukf); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 3 prob rob ( Predict ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sps_pred = zeros(dim_state, num_sp);
    for sp_ind = 1:num_sp
        sps_pred(:, sp_ind) = propagate_state(sps(:, sp_ind), model, u);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 4 & 5 - predict new mean & cov %%%%%%%%%%%%%%%%%%%%%%%
    [mu_bar, sigma_bar, Wprime] = predict_mean_and_cov_state(sps_pred, yukf, yukf.prms.R);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 6 - update sigma points 
    sps_updated = calc_sigma_points(mu_bar, sigma_bar, yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 7 - 9 - predict observation & uncertainty for each sigma point
    pred_obs = predict_obs(sps_updated, camera, initial_bb, yukf);
    [z_hat, S, S_inv] = predict_mean_and_cov_obs(pred_obs, yukf, yukf.prms.Q);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 10 - calc cross correlation
    sigma_xz = calc_cross_correlation(Wprime, z_hat, pred_obs, yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 11 - 13: kalman gain & update
    K = sigma_xz * S_inv;
    b_add_all = false;
    if b_add_all
        % this is how the paper I am following does it
        mu_out = mu_bar + K * (z - z_hat);
    else
        % this is what would make sense to me
        innovation = K * (z - z_hat);
        mu_out(1:9) = mu_bar(1:9) + innovation(1:9);
        mu_out(10:13) = quatmultiply(mu_bar(10:13)', axang_to_quat(innovation(10:12))' );
    end
    sigma_out = sigma_bar - K * S * K';
    
    % project sigma to pos. def. cone to avoid numeric issues
    [V, D] = eig(sigma_out);
    D(D < 0) = 0.000001;
    sigma_out = V * D * V';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    yukf.mu_prev = yukf.mu;
    yukf.mu = mu_out(:);
    yukf.sigma = sigma_out;
    disp('')
end