function yukf = yukf_step(yukf, u, z, model, camera, initial_bb)
    global flight k_act t_tmp k
    if isempty(k_act)
        k_act = k;
    end
    dim = length(yukf.mu);
    dim_cov = length(yukf.sigma);
    num_sp = 2*dim_cov + 1;
    
    % line 2 prob rob
    sps = calc_sigma_points(yukf.mu, yukf.sigma, yukf); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 3 prob rob ( Predict ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sps_pred = zeros(dim, num_sp);
    for sp_ind = 1:num_sp
        sps_pred(:, sp_ind) = propagate_state(sps(:, sp_ind), model, u, yukf.dt);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 4 & 5 - predict new mean & cov %%%%%%%%%%%%%%%%%%%%%%%
    [mu_bar, sigma_bar] = predict_mean_and_cov_state(sps_pred, yukf, yukf.prms.Q);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 6 - update sigma points 
    sps_updated = calc_sigma_points(mu_bar, sigma_bar, yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 7 - 9 - predict observation & uncertainty for each sigma point
    pred_obs = predict_obs(sps_updated, camera, initial_bb, yukf);
    [z_hat, S, S_inv] = predict_mean_and_cov_obs(pred_obs, yukf, yukf.prms.R);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 10 - calc cross correlation
    sigma_xz = calc_cross_correlation(sps_updated, mu_bar, z_hat, pred_obs, yukf);
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
        mu_out(1:6) = mu_bar(1:6) + innovation(1:6);
        mu_out(11:13) = mu_bar(11:13) + innovation(10:12);
%         q_tmp = quatmultiply(mu_bar(7:10)', axang_to_quat(innovation(7:9))' );
        q_tmp = quatmultiply(axang_to_quat(innovation(7:9))', mu_bar(7:10)');
        mu_out(7:10) = q_tmp;
        
        if any([yukf.prms.b_enforce_0_yaw, yukf.prms.b_enforce_yaw, yukf.prms.b_enforce_pitch, yukf.prms.b_enforce_roll])
            q_tmp = cheat_with_angles(q_tmp);
            q_tmp = normalize_quat(q_tmp);
            mu_out(7:10) = q_tmp;
        end
    end
    sigma_out = sigma_bar - K * S * K';
    
    % project sigma to pos. def. cone to avoid numeric issues
    sigma_out = enforce_pos_def_sym_mat(sigma_out);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    yukf.mu_prev = yukf.mu;
    yukf.mu = mu_out(:);
    yukf.sigma = sigma_out;
    disp('')
end