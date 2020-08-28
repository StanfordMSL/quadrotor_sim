function [fp_flag,stab_flag] = fp_flag_check(Jc,Jp,alpha,del_V)

% Regularization check
if isnan(Jc)
    % Get out of forward pass. We need to restart from backward pass.
    stab_flag = false;
    fp_flag = true; 
else
    stab_flag = true;

    % Expected Improvement check
    z = (Jc-Jp)/(alpha*sum(del_V(1,:)) + (alpha^2)*sum(del_V(2,:)));

    if (z > 1e-4) && (z < 10)
        fp_flag = true;
    elseif alpha < 1e-5
        fp_flag = true;
    else
        fp_flag = false;
    end
end

end