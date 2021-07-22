function [flag,alpha] = check_LS(Jc,Jp,alpha,delV)

tol_alpha = 0.00001;
z = Jp.tot-Jc.tot;
% z = Jp.con-Jc.con;

if (z > 0)      
    % Constraint Cost Improved. Allow a Trajectory Update
    flag = 0;
else
    % Constraint Cost Worsened.
    if alpha(1,1) > tol_alpha        
        % Try simply reducing alpha.
        flag = 1;
        alpha = 0.5.*alpha; 
    elseif (alpha(1,1) < tol_alpha)  
        % Reducing alpha not helping. Resorting to alternatives.
        flag = 2;
    end
end   
