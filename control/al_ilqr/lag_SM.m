function flag = lag_SM(La_c,La_p,alpha)

tol_obj = 1e-2;
tol_con = 1e-2;

del_obj = La_p.obj - La_c.obj;
del_con = La_p.con - La_c.con;

% if (alpha == 0)                                 % alpha was zero'd
%     flag = 0;                                       % = resort to multiplier update
% else
%     if (del_con > tol_con)                      % good constraint improvement
%         flag = 1;                                   % = use alpha and continue iLQR
%     elseif ((del_con>0) && (del_con<tol_con))   % mediocre constraint improvement
%         if (del_obj  > tol_obj)                 % good objective improvement
%             flag = 1;                               % = use alpha and continue iLQR
%         else
%             flag = 0;                           % mediocre objective improvement
%         end                                         % = iLQR tapered. go for multiplier update
%     else                                        % constraint worsened
%         flag = 2;                                   % = update alpha
%     end
% end

if (alpha == 0)                                 % alpha was zero'd
    flag = 0;                                       % = resort to multiplier update
else
    if (del_con < tol_con) && (del_obj < tol_obj)
        flag = 0;
    else
        flag = 1;
    end
end

end