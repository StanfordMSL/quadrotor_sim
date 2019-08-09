function [x_bar, u_bar, del_u, A, B] = ilqr_fp(x_bar,u_bar,t_now,x_fc,wp,l,L,N,model,fc)

% Unpack some terms
t_wp = wp.t;
x_wp = wp.x;
Q_key = wp.Q_key;

err_x = zeros(12,wp.N_wp);

% Initialize some terms
cost_curr = 1;
cost_prev = 0;
alpha = 2;

del_x = zeros(12,N-1);
del_u = zeros(4,N-1);

A = zeros(12,12,N);
B = zeros(12,4,N);

while cost_curr > cost_prev 
    cost_prev = cost_curr;
    cost_curr = 0;
    alpha = 0.5*alpha;

    x_fp = zeros(12,N);
    x_fp(:,1) = x_fc;
    
    u_test = u_bar;
    
    wp_k = 2;
    for k = 1:N-1
        t_fp = k*model.fc_dt+t_now;
        
        % Determine Control Command
        del_x(:,k) = x_fp(:,k)-x_bar(:,k);
        del_u(:,k) = alpha*l(:,:,k) + L(:,:,k)*del_x(:,k);
        u_test(:,k) = u_test(:,k) + del_u(:,k);

        % Determine A and B matrices for this step
        A(:,:,k) = A_calc_wrench(x_fp(:,k),u_test(:,k),model);
        B(:,:,k) = B_calc_wrench(x_fp(:,k),model);
             
        % Predict Dynamics of Next Step
        FT_ext = zeros(6,1);
        m_cmd = wrench2m_cmd(u_test(:,k),model);

        x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'fc');
        
        % Update cost (used to break loop)
        if (wp_k < wp.N_wp) && (abs(t_fp - t_wp(wp_k)) <= model.fc_dt)
            Q = fc.Q(:,:,Q_key(wp_k));
            R = fc.R;
            
            err_x(:,wp_k) = x_fp(:,k)-x_wp(wp_k);
            cost_curr = cost_curr + 0.5*(err_x'*Q*err_x + u_test(:,k)'*R*u_test(:,k));
            wp_k = wp_k + 1;
        else
            Q = fc.Q(:,:,1);
            R = fc.R;
            
            cost_curr = cost_curr + 0.5*(x_fp(:,k)'*Q*x_fp(:,k) + u_test(:,k)'*R*u_test(:,k));
        end 
    end
    
    if alpha < 1e-6
        disp('[ilqr_fp]: PROBLEM! alpha too small!');
    end
    
    % Add terminal cost
    err_x(:,wp_k) = x_fp(:,N)-x_wp(wp_k);
    cost_curr = cost_curr + 0.5*err_x'*fc.Q(:,:,end)*err_x;
end

% If cost goes down, we know it's feasible. Update x_bar.
x_bar = x_fp;
u_bar = u_test;

end