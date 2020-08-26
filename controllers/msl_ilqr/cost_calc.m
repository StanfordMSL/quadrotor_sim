function J = cost_calc(x_bar,u_bar,con,lambda,I_mu,cost_param)
    % Unpack some stuff
    x_star = cost_param.x_star;
    u_star = cost_param.u_star;   
    Q      = cost_param.Q;
    R      = cost_param.R;
    
    % Count
    N = size(x_bar,2);
    
    % Initialize cost variables
    J_lqr_arr = zeros(1,N);
    J_aug_arr = zeros(1,N);
    
    % Compute stagewise cost
    for k = 1:N-1
        err_x = x_bar(:,k) - x_star;
        err_u = u_bar(:,k) - u_star;
        J_lqr_arr(1,k) = 0.5* err_x'*Q(:,:,k)*err_x + 0.5*err_u'*R(:,:,k)*err_u;
        J_aug_arr(1,k) = (lambda(:,k) +  0.5.*I_mu(:,:,k)*con(:,k))'*con(:,k);
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_star;
    J_lqr_arr(1,N) = 0.5* err_x'*Q(:,:,N)*err_x;
    J_aug_arr(1,N) = (lambda(:,N) +  0.5.*I_mu(:,:,N)*con(:,N))'*con(:,N);

    % Totals
    J.lqr = sum(J_lqr_arr);
    J.aug = sum(J_aug_arr);
    J.tot = J.lqr + J.aug;
end