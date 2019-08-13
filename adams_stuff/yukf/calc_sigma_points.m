function sps = calc_sigma_points(mu_curr, sig_curr, ukf_prms)
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    dim = size(mu_curr,1);
    sps = zeros(dim, 2*dim + 1);
    
    sps(:, 1) = mu_curr;
    sigma_point_step = sqrt(dim + ukf_prms.lambda) * sqrtm(sig_curr);
    
    q_mean = mu_curr(7:9); 
    q_mean = [sqrt(1 - q_mean'*q_mean); q_mean];
    for i = 1:dim
        % first the + peturb
        sps(1:6, 2 + 2*(i-1))  = mu_curr(1:6) + sigma_point_step(1:6, i);
        sps(10:12, 2 + 2*(i-1)) = mu_curr(10:12) + sigma_point_step(10:12, i);
        
        q_perturb = calc_peturb_quat(sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:9, 2 + 2*(i-1)) = q_new(2:4);
        
        % now the - peturb
        sps(1:6, 2 + 2*(i-1) + 1) = mu_curr(1:6) - sigma_point_step(1:6, i);
        sps(10:12, 2 + 2*(i-1) + 1) = mu_curr(10:12) - sigma_point_step(10:12, i);
        
        q_perturb = calc_peturb_quat(-sigma_point_step(7:9, i));
        q_new = quatmultiply(q_mean(:)', q_perturb(:)');
        sps(7:9, 2 + 2*(i-1) + 1) = q_new(2:4);
    end
end