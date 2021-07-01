function [lambda,mu] = mult_update(lambda,mu,phi,con)

n_con = size(lambda,1);
for k = 1:size(con,2)
    lambda_cand = lambda(:,k) + mu(:,k).*con(:,k);

    lambda(:,k) = max(zeros(n_con,1),lambda_cand);
    mu(:,k) = phi.*mu(:,k);
end