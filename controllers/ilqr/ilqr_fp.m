function [x_bar,u_bar,u_diff] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,model,fc)

% Unpack some terms
t_wp = wp.t;
x_wp = wp.x;
Q_key = wp.Q_key;

% Initialize some terms
alpha = 1;
x_fp = zeros(12,N);
x_fp(:,1) = x_now;
u_fp = u_bar; 
wp_k = 2;
Q = fc.Q(:,:,Q_key(wp_k));
R = fc.R;
x_targ = x_wp(:,wp_k);

cost_curr =  0;
    
for k = 1:N-1
    t_fp = t_bar(k);

    % Determine Control Command
    del_x = x_fp(:,k)-x_bar(:,k);
    del_u = alpha*l(:,:,k) + L(:,:,k)*del_x;
    u_fp(:,k) = u_fp(:,k) + del_u;

    % Predict Dynamics of Next Step
    FT_ext = zeros(6,1);
    m_cmd = wrench2m_controller(u_fp(:,k),model);

    x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'fc');

    % Trigger Q, R and target waypoint update
    if (abs(t_fp - t_wp(wp_k)) < model.con_dt) && (wp_k < (wp.N_wp))
        wp_k = wp_k + 1;

        Q = fc.Q(:,:,Q_key(wp_k));
        x_targ = x_wp(:,wp_k);
    else
%             Q = fc.Q(:,:,Q_key(1));
%             x_targ = zeros(12,1);
    end

    % Update Cost
    err_x = x_fp(:,k)-x_targ;
    cost_curr = cost_curr + 0.5*(err_x'*Q*err_x + u_fp(:,k)'*R*u_fp(:,k));
end

% Add terminal cost   
Q = fc.Q_N;
err_x = x_fp(:,N)-x_targ;
cost_curr = cost_curr + 0.5* err_x'*Q*err_x;

% disp(['[ilq_fp]: Current Cost: ',num2str(cost_curr)]);

% If cost goes down, we know it's feasible. Update x_bar.
u_diff = sum(vecnorm(u_bar-u_fp));
x_bar = x_fp;
u_bar = u_fp;

end