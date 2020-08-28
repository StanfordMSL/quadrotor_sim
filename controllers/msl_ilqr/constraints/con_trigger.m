function I_mu = con_trigger(con,lambda,mu)
    
N = size(con,2);
n_con = size(con,1);
I_mu = zeros(n_con,n_con,N);

for k = 1:N
    for j = 1:16
        if con(j,k) <= 0 && lambda(j,k) == 0
            % Constraint not violated. Carry on.
        else
            % Constraint violated. Turn on Augment.
            I_mu(j,j,k) = mu(j,k);
        end
    end
end

end