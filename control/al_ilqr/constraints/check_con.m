function mud = check_con(con,lambda,mu,tol)

n_c = size(con,1);
N   = size(con,2);

mud = zeros(n_c,N);

for k = 1:N
    for j = 1:n_c
        if con(j,k) <= tol && lambda(j,k) == 0
            % Constraint not violated. Carry on.
        else
            % Constraint violated. Turn on Augment.
            mud(j,k) = mu(j,k);
        end
    end
end
end
