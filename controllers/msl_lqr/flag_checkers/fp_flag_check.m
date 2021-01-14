function [fp_flag,stab_flag] = fp_flag_check(Jc,Jp,alpha,del_V)

% Regularization check
if isnan(Jc)
    % Get out of forward pass. We need to restart from backward pass.
    stab_flag = false;
    fp_flag = true; 
    disp('[fp_flag_check]: Cost exploded. Restarting from bp with higher rho');
else
    % Expected Improvement check
    z = (Jc-Jp)/(alpha*sum(del_V(1,:)) + (alpha^2)*sum(del_V(2,:)));

    if (z > 1e-4) && (z < 10)
        stab_flag = true;
        fp_flag = true;
%         disp('[fp_flag_check]: Cost improvement close enough to predicted.');
    elseif alpha < 1e-10
        stab_flag = true;
        fp_flag = true;
%         disp('[fp_flag_check]: Alpha is now negligible.');
    else
        stab_flag = true;
        fp_flag = false;
%         disp('[fp_flag_check]: Cost improvement too far from predicted, reducing alpha.');
    end
end

end