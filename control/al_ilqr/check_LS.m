function [flag,alpha] = check_LS(Jc,Jp,alpha,delV)

tol_alpha = 0.3;
z = Jp.tot-Jc.tot;

if (z > 0)
    % Line-Search Gain Valid
    flag = 0;
elseif ( (z <= 0) && (z > -100))
    if alpha > tol_alpha
        % Line-Search Gain Invalid (too big). Reduce it more.
        flag = 1;
        alpha = 0.5.*alpha; 
    elseif (alpha < tol_alpha)
        % Alpha Too Small. Generate an 'empty' update
        flag = 2;
    end
else    
    % Trajectory Exploded. Generate an 'empty' update
    flag = 2;
end