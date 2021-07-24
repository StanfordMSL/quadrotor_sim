function flag = check_inner(Jc,Jp,tol_inner)
    
delJ = Jp.tot-Jc.tot;
if (delJ > tol_inner)
    % Constrained cost improved. Allow more interations with current
    % Lagrange parameters.
    flag = 0;
elseif (delJ < tol_inner) && (delJ >= 0)
    % Constrained cost improved but it seems like we need to update the
    % Lagrange. The lemon has been fully squeezed.
    flag = 1;
else
    % Constrained cost worsened. We try a Lagrange update to hopefully stop
    % this.
    flag = 2;
end
% disp(['[check_inner]: Cost Difference: ',num2str(delJ)]);
