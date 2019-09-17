function sps = calc_sigma_points(mu, sigma, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    dim = size(mu, 1);
    sps = zeros(dim, 2 * dim + 1);
    
    sps(:, 1) = mu;
    sig_sqrt = chol(sigma); 
%     sig_sqrt = sqrtm(sigma);
%     [U, S, V] = svd(sigma);
%     sig_sqrt = U * S.^0.5 * V';
    sigma_point_step = sqrt(dim + yukf.prms.lambda) * sig_sqrt; 
    if yukf.prms.b_enforce_0_yaw
        sigma_point_step(12, :) = 0;
    end
    
    q_mean = complete_unit_quat(mu(7:9)); 
    for i = 1:dim
        % first the + peturb
        sps(1:6, 2 + 2*(i-1))  = mu(1:6) + sigma_point_step(1:6, i);
        sps(10:12, 2 + 2*(i-1)) = mu(10:12) + sigma_point_step(10:12, i);
        
        q_perturb = axang_to_quat(sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:9, 2 + 2*(i-1)) = q_new(2:4);
        
        % now the - peturb
        sps(1:6, 2 + 2*(i-1) + 1) = mu(1:6) - sigma_point_step(1:6, i);
        sps(10:12, 2 + 2*(i-1) + 1) = mu(10:12) - sigma_point_step(10:12, i);
        
        q_perturb = axang_to_quat(-sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:9, 2 + 2*(i-1) + 1) = q_new(2:4);
    end
end