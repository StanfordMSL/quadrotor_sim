function al = al_update(al)

for k = 1:size(al.con,2)
    lambda_cand = al.lambda(:,k) + al.mu(:,k).*al.con(:,k);

    al.lambda(:,k) = max(zeros(22,1),lambda_cand);
    al.mu(:,k) = al.phi.*al.mu(:,k);
end

end