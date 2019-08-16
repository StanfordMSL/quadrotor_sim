function sps = calc_sigma_points(mu, sigma, yukf)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    dim_state = size(mu, 1);
    dim_cov = size(yukf.sigma, 1);
    num_sp = 2 * dim_cov + 1;
    sps = zeros(dim_state, num_sp);
    
    sps(:, 1) = mu;
    sig_sqrt = chol(sigma); 
%     sig_sqrt = sqrtm(sigma);
%     [U, S, V] = svd(sigma);
%     sig_sqrt = U * S.^0.5 * V';
    sigma_point_step = sqrt(dim_cov + yukf.prms.lambda) * sig_sqrt; 
    
    q_mean = mu(10:13); 
    for i = 1:dim_cov        
        sps(1:9, 2 + 2*(i-1))     = mu(1:9) + sigma_point_step(1:9, i); % first the + peturb
        sps(1:9, 2 + 2*(i-1) + 1) = mu(1:9) - sigma_point_step(1:9, i); % then the - peturb
        
        q_perturb = axang_to_quat(sigma_point_step(10:12, i)); % first the + peturb
        sps(10:13, 2 + 2*(i-1)) = quatmultiply(q_mean(:)', q_perturb(:)')';
        
        q_perturb = axang_to_quat(-sigma_point_step(10:12, i)); % then the - peturb
        sps(10:13, 2 + 2*(i-1) + 1) = quatmultiply(q_mean(:)', q_perturb(:)')';
    end
end