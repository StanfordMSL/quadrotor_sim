function [flag,alpha] = check_LS(Jc,Jp,alpha,delV)

z = Jp.tot-Jc.tot;
flag = 999;

if (z > 0)
    % Line-Search Gain Valid
    flag = 0;
else
    if z < -100
        % Trajectory Exploded. Generate an 'empty' update
        flag = 2;
    end
    if alpha > 0.3
        % Line-Search Gain Invalid
        flag = 1;
        alpha = 0.5.*alpha; 
    elseif ((alpha < 0.3) && (alpha > 0.0))
        % Alpha Too Small. Generate an 'empty' update
        flag = 2;
    end
end