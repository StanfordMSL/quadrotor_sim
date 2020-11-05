function t_vect = t_vect_compute(t_now,t_dim)
    t_vect = zeros(t_dim,1);

    for k = 1:t_dim
        t_vect(k,1) = (t_now^(k-1))/factorial(k-1);
    end
end