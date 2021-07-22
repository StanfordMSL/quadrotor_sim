function mult = mult_update(mult,con)

n_con = size(mult.lambda,1);
for k = 1:size(con,2)
    lambda_cand = mult.lambda(:,k) + mult.mu(:,k).*con(:,k);

    mult.lambda(:,k) = max(zeros(n_con,1),lambda_cand);
    mult.mu(:,k) = mult.phi.*mult.mu(:,k);
end