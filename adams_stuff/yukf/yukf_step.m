function yukf = yukf_step(yukf, u, z, model, camera, initial_bb)
    global flight k_act t_tmp k
    dim = length(yukf.mu);
    num_sp = 2*dim + 1;
    
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
        mu_out(1:6) = mu_bar(1:6) + innovation(1:6);
        mu_out(10:12) = mu_bar(10:12) + innovation(10:12);
        q_tmp = quatmultiply( complete_unit_quat(mu_bar(7:9))', axang_to_quat(innovation(7:9))' );
        mu_out(7:9) = q_tmp(2:4);
        if yukf.prms.b_enforce_0_yaw
            mu_out(9) = 0; % can probably do a better job forcing yaw to 0 (convert to eul, zero yaw, convert back)
        end
    end
%     mu_out = mu_bar; % DEBUG1
    sigma_out = sigma_bar - K * S * K';
    
    % project sigma to pos. def. cone to avoid numeric issues
    [V, D] = eig(sigma_out);
    D(D < 0) = 0.000001;
    sigma_out = V * D * V';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if isempty(k_act)
        k_act = k;
    end
    err = flight.x_act(:, k_act) - mu_out(:);
    [y1, p1, r1] = quat2angle(complete_unit_quat(flight.x_act(7:9, k_act))');
    [y2, p2, r2] = quat2angle(complete_unit_quat(mu_out(7:9))');
    err_ypr = [y1 - y2, p1 - p2, r1 - r2] * 180/pi;
    
    yukf.mu_prev = yukf.mu;
    yukf.mu = mu_out(:);
    yukf.sigma = sigma_out;
    disp('')
end