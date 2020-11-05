function H = H_build(tf,n_der,k_min)
    % Build the H matrix integral
    N_T = n_der - k_min;
    t_arr = zeros(N_T*2-1,1);
    for k = 1:N_T*2-1
        t_arr(k,1) = (1/k) * (tf^k);
    end

    H_base = zeros(N_T,N_T);
    for k = 1:(N_T*2-1)
        m = N_T-abs(N_T-k);
        feed = t_arr(k,1).*ones(m,1);

        idx = N_T-k;
        H_base = H_base + diag(feed,idx);
    end
    H_base = flip(H_base,2);

    h_fact = zeros(N_T,1);
    for k = 1:N_T
        h_fact(k,1) = 1./factorial(k-1);
    end
    H_fact = h_fact*h_fact';

    H = 1e-12.*eye(n_der);
    H(k_min+1:end,k_min+1:end) = H_base./H_fact;
end