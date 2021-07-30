function flag = check_LS(La_c,La_p)

tol_obj = 1;
tol_con = 1e-2;

del_obj = La_p.obj - La_c.obj;
del_con = La_p.con - La_c.con;

if (abs(del_con) < tol_con)         % Constraint Moderate
    if (abs(del_obj) < tol_obj)         % Cost Moderate
        flag = 0;                           % Use alpha and update multiplier
    elseif (del_obj > tol_obj)          % Cost Improved
        flag = 1;                           % Use alpha and maintain multiplier
    else                                % Cost Worsened
        flag = 1;                           % Update alpha
    end
elseif (del_con > tol_con)          % Constraint Improved
    if (del_obj > -tol_obj)             % Cost Moderate/Improved
        flag = 1;                           % Use alpha and maintain multiplier
    else                                % Cost Worsened
        flag = 2;                           % Update alpha
    end
else                                 % Constraint Worsened
    flag = 2;                           % Update alpha.
end

end