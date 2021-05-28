function I_mu = check_con(con,lambda,mu)

n_c = size(con,1);
N = size(con,2);

I_mu = zeros(n_c,N);

for k = 1:N
    for j = 1:n_c
        if con(j,k) <= 0 && lambda(j,k) == 0
            % Constraint not violated. Carry on.
        else
            % Constraint violated. Turn on Augment.
            I_mu(j,k) = mu(j,k);
        end
    end
end
end
