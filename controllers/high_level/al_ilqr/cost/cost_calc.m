function J = cost_calc(x_bar,u_bar,con,lambda,I_mu,cost_param)
    % Unpack some stuff
    x_k = cost_param.x_k;
    u_k = cost_param.u_k;   
    Q_k    = cost_param.Q_k;
    R_k    = cost_param.R_k;
    Q_N    = cost_param.Q_N;
    
    % Count
    N = size(x_bar,2);
    
    % Initialize cost variables
    J_lqr_arr = zeros(1,N);
    J_aug_arr = zeros(1,N);
    
    % Compute stagewise cost
    for k = 1:N-1
        err_x = x_bar(:,k) - x_k(:,k);
        err_u = u_bar(:,k) - u_k(:,k);
        J_lqr_arr(1,k) = 0.5* err_x'*Q_k(:,:,k)*err_x + 0.5*err_u'*R_k(:,:,k)*err_u;
        J_aug_arr(1,k) = (lambda(:,k) +  0.5.*I_mu(:,:,k)*con(:,k))'*con(:,k);
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_k(:,k);
    J_lqr_arr(1,N) = 0.5* err_x'*Q_N*err_x;
    J_aug_arr(1,N) = (lambda(:,N) +  0.5.*I_mu(:,:,N)*con(:,N))'*con(:,N);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     K = 1e-9;
%     
%     q_N = x_bar(7:10,end);
%     q_star = x_star(7:10,1);
%     err_geo = (acos(q_N'*q_star))^4;
%     J_lqr_arr(1,N) = J_lqr_arr(1,N) + K.*err_geo;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Totals
    J.lqr = sum(J_lqr_arr);
    J.aug = sum(J_aug_arr);
    J.tot = J.lqr + J.aug;
end