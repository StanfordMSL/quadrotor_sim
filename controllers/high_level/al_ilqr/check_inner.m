function flag = check_inner(Jc,Jp,tol_inner)
    
delJ = Jp.con-Jc.con;
if (delJ > tol_inner)
    flag = 0;           % keep going
elseif delJ < 0
    flag = 1;           % stop and use previous
else
    flag = 2;           % stop at current
end
% disp(['[check_inner]: Cost Difference: ',num2str(delJ)]);
