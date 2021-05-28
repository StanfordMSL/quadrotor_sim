function flag = check_LS(Jc,Jp,alpha,delV)

z = (Jc.tot-Jp.tot)/(alpha*sum(delV(1,:)) + (alpha^2)*sum(delV(2,:)));

if (z > 1e-4) && (z < 10)
    flag = true;
elseif alpha < 1e-3
    flag = true;
else
    flag = false;
end