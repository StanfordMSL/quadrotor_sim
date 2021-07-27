function mult = mult_update(mult,con,phi)

conx = con(1:16,:);
conu = con(17:24,:);

nx = size(mult.lamx,1);
nu = size(mult.lamu,1);

for k = 1:size(conx,2)
    lambda_cand = mult.lamx(:,k) + mult.mux(:,k).*conx(:,k);

    mult.lamx(:,k) = max(zeros(nx,1),lambda_cand);
    mult.mux(:,k)  = phi.*mult.mux(:,k);
end

for k = 1:size(conu,2)
    lambda_cand = mult.lamu(:,k) + mult.muu(:,k).*conu(:,k);

    mult.lamu(:,k) = max(zeros(nu,1),lambda_cand);
    mult.muu(:,k)  = phi.*mult.muu(:,k);
end

mult.mudx = check_con(conx,mult.lamx,mult.mux,0);
mult.mudu = check_con(conu,mult.lamu,mult.muu,0);