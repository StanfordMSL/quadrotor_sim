function al = al_update(al)

n_con = size(al.lambda,1);
for k = 1:size(al.con,2)
    lambda_cand = al.lambda(:,k) + al.mu(:,k).*al.con(:,k);

    al.lambda(:,k) = max(zeros(n_con,1),lambda_cand);
    al.mu(:,k) = al.phi.*al.mu(:,k);
end

end