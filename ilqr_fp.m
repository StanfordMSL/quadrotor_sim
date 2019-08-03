function [x_bar, u_bar, del_u, A, B] = ilqr_fp(x_bar,u_bar,x_fc,l,L,N,model,fc)

% Unpack some terms
Q_lqr = fc.Q;
R_lqr = fc.R;
Q_N   = fc.Q_N;

% Initialize some terms
cost_curr = 1e6;
cost_prev = 0;
alpha = 2;

x_fp = zeros(12,N+1);
x_fp(:,1) = x_fc;

del_x = zeros(12,N);
del_u = zeros(4,N);

A = zeros(12,12,N);
B = zeros(12,4,N);
    
while cost_curr > cost_prev 
    cost_prev = cost_curr;
    cost_curr = 0;
    alpha = 0.5*alpha;

    for k = 1:N
        % Determine Control Command
        del_x(:,k) = x_fp(:,k)-x_bar(:,k);
        del_u(:,k) = alpha*l(:,:,k) + L(:,:,k)*del_x(:,k);
        u_bar(:,k) = u_bar(:,k) + del_u(:,k);

        % Determine A and B matrices for this step
        A(:,:,k) = A_calc_wrench(x_fp(:,k),u_bar(:,k),model);
        B(:,:,k) = B_calc_wrench(x_fp(:,k),model);
             
        % Predict Dynamics of Next Step
        FT_ext = zeros(6,1);
        m_cmd = wrench2m_cmd(u_bar(:,k),model);

        x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'fc');
        
        % Update cost (used to break loop)
        cost_curr = cost_curr + 0.5*(x_fp(:,k)'*Q_lqr*x_fp(:,k) + u_bar(:,k)'*R_lqr*u_bar(:,k));
    end
    
    if alpha < 1e-6
        disp('[ilqr_fp]: PROBLEM! alpha too small!');
    end
    % Add terminal cost
    cost_curr = cost_curr + 0.5*x_bar(:,k)'*Q_N*x_bar(:,k);
end

% If cost goes down, we know it's feasible. Update x_bar.
x_bar = x_fp;

end