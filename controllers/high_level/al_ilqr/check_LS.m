function flag = check_LS(Jc,Jp,alpha,delV)

z = (Jc.tot-Jp.tot)/(alpha*sum(delV(1,:)) + (alpha^2)*sum(delV(2,:)));

if (z < 10)
% if (z > 1e-4) && (z < 10)
% if (z > -1) && (z < 10)
% if (z > -10) && (z < 10)
    flag = true;
else
    flag = false;
end