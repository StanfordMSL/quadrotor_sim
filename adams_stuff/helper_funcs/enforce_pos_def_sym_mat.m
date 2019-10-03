function sigma_out = enforce_pos_def_sym_mat(sigma)
    sigma_out = (sigma + sigma')/2;
    [V, D] = eig(sigma_out);
    D(D < 0) = 0.000001;
    sigma_out = V * D * V';
end