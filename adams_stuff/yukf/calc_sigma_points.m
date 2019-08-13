function sps = calc_sigma_points(mu_curr, sig_curr, ukf_prms)
    dim = size(mu_curr,1);
    sps = zeros(dim, 2*dim + 1);
    
    sps(:, 1) = mu_curr;
    sigma_point_step = sqrt(dim + ukf_prms.lambda) * sqrtm(sig_curr);
    for i = 1:dim
        sps(:, 2 + 2*(i-1))     = mu_curr(:) + sigma_point_step(:, i);
        sps(:, 2 + 2*(i-1) + 1) = mu_curr(:) - sigma_point_step(:, i);
    end
end