function [J,J_stg,J_aug] = J_calc(x_star,x_bar,u_bar,al,wts)
    % Unpack some stuff
    N = size(x_bar,2);
    
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    
    % Initialize cost variables
    J_stg = zeros(1,N);
    J_aug = zeros(1,N);
    
    % Compute stagewise cost
    for k = 1:N-1
        err_x = x_bar(:,k) - x_star;
        err_u = u_bar(:,k);
        J_stg(1,k) = 0.5* err_x'*Q_t*err_x + 0.5*err_u'*R*err_u;
        J_aug(1,k) = (al.lambda(:,k) +  0.5.*al.I_mu(:,:,k)*al.con(:,k))'*al.con(:,k);
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_star;
    J_stg(1,N) = 0.5* err_x'*Q_f*err_x;
    J_aug(1,N) = (al.lambda(:,N) +  0.5.*al.I_mu(:,:,N)*al.con(:,N))'*al.con(:,N);

    % Total
    J = sum(J_stg) + sum(J_aug);
end