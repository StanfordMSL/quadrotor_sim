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
    for i = 1:dim_sigma
        % first the + peturb
        sps(1:6, 2 + 2*(i-1))  = mu(1:6) + sigma_point_step(1:6, i);
        sps(11:13, 2 + 2*(i-1)) = mu(11:13) + sigma_point_step(10:12, i);
        
        q_perturb = axang_to_quat(sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:10, 2 + 2*(i-1)) = q_new;
        
        % now the - peturb
        sps(1:6, 2 + 2*(i-1) + 1) = mu(1:6) - sigma_point_step(1:6, i);
        sps(11:13, 2 + 2*(i-1) + 1) = mu(11:13) - sigma_point_step(10:12, i);
        
        q_perturb = axang_to_quat(-sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:10, 2 + 2*(i-1) + 1) = q_new;
    end
end