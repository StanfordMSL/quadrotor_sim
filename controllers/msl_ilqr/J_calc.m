function J = J_calc(x_star,u_star,x_bar,u_bar,al,wts)
    % Unpack some stuff
    N = size(x_bar,2);
    
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    
    % Initialize cost variables
    J_stg_arr = zeros(1,N);
    J_aug_arr = zeros(1,N);
    
    % Compute stagewise cost
    for k = 1:N-1
        err_x = x_bar(:,k) - x_star;
        err_u = u_bar(:,k) - u_star;
        J_stg_arr(1,k) = 0.5* err_x'*Q_t*err_x + 0.5*err_u'*R*err_u;
        J_aug_arr(1,k) = (al.lambda(:,k) +  0.5.*al.I_mu(:,:,k)*al.con(:,k))'*al.con(:,k);
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_star;
    J_stg_arr(1,N) = 0.5* err_x'*Q_f*err_x;
    J_aug_arr(1,N) = (al.lambda(:,N) +  0.5.*al.I_mu(:,:,N)*al.con(:,N))'*al.con(:,N);

    % Totals
    J.stg = sum(J_stg_arr);
    J.aug = sum(J_aug_arr);
    J.tot = J.stg + J.aug;
end