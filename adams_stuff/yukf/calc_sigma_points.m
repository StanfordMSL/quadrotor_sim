function sps = calc_sigma_points(mu, sigma, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    dim_sigma = length(yukf.sigma);
    dim = size(mu, 1);
    sps = zeros(dim, 2 * dim_sigma + 1);
    
    sps(:, 1) = mu;
    sig_sqrt = chol(sigma); 
%     sig_sqrt = sqrtm(sigma);
%     [U, S, V] = svd(sigma);
%     sig_sqrt = U * S.^0.5 * V';
    sigma_point_step = sqrt(dim_sigma + yukf.prms.lambda) * sig_sqrt; 
    if yukf.prms.b_enforce_0_yaw
        sigma_point_step(9, :) = 0;
    end
    
    q_mean = mu(7:10); 
    q_mean_ego = mu(20:23); 
    for i = 1:dim_sigma
        % first the + peturb
        sps(1:6, 2 + 2*(i-1))  = mu(1:6) + sigma_point_step(1:6, i);
        sps(11:13, 2 + 2*(i-1)) = mu(11:13) + sigma_point_step(10:12, i);
        sps(14:19, 2 + 2*(i-1)) = mu(14:19) + sigma_point_step(13:18, i);
        sps(24:26, 2 + 2*(i-1)) = mu(24:26) + sigma_point_step(22:24, i);
        
        q_perturb = axang_to_quat(sigma_point_step(7:9, i));
        q_new = quatmultiply(q_perturb(:)', q_mean(:)');
        sps(7:10, 2 + 2*(i-1)) = q_new;
        
        q_perturb = axang_to_quat(sigma_point_step(19:21, i));
        q_new = quatmultiply(q_perturb(:)', q_mean_ego(:)');
        sps(20:23, 2 + 2*(i-1)) = q_new;
        
        % now the - peturb
        sps(1:6, 2 + 2*(i-1) + 1) = mu(1:6) - sigma_point_step(1:6, i);
        sps(11:13, 2 + 2*(i-1) + 1) = mu(11:13) - sigma_point_step(10:12, i);
        sps(14:19, 2 + 2*(i-1) + 1) = mu(14:19) - sigma_point_step(13:18, i);
        sps(24:26, 2 + 2*(i-1) + 1) = mu(24:26) - sigma_point_step(22:24, i);
        
        q_perturb = axang_to_quat(-sigma_point_step(7:9, i));
        q_new = quatmultiply(q_perturb(:)', q_mean(:)');
        sps(7:10, 2 + 2*(i-1) + 1) = q_new;
        
        q_perturb = axang_to_quat(-sigma_point_step(19:21, i));
        q_new = quatmultiply(q_perturb(:)', q_mean_ego(:)');
        sps(20:23, 2 + 2*(i-1) + 1) = q_new;
    end
end
