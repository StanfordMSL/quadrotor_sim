function mu_d = fp_mud_upd(con,lambda,mu)

mu_d = zeros(24,1);

for j = 1:24
    if con(j,1) <= 0 && lambda(j,1) == 0
        % Constraint not violated. Carry on.
    else
        % Constraint violated. Turn on Augment.
        mu_d(j,1) = mu(j,1);
    end
end