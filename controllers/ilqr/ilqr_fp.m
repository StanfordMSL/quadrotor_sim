function [x_bar,u_bar,u_diff] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,alpha,model,fc)

% Unpack some terms
t_wp = wp.t;
x_wp = wp.x;
Q_key = wp.Q_key;

% Find wp to start from
wp_fw = find((t_wp > t_bar(1)),1)-1;       % This is the waypoint we are seeking. 
R = fc.R;

% Initialize some terms
x_fp = zeros(12,N);
x_fp(:,1) = x_now;
u_fp = u_bar; 
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
    if ((t_fp + model.con_dt) > t_wp(wp_fw+1)) && (wp_fw < (wp.N_wp))
        Q = fc.Q(:,:,Q_key(wp_fw,2));

        wp_fw = wp_fw + 1;      
    else
        Q = fc.Q(:,:,Q_key(wp_fw,1));
        x_targ = x_wp(:,wp_fw+1);
    end

    % Update Cost
    err_x = x_fp(:,k)-x_targ;
    cost_curr = cost_curr + 0.5*(err_x'*Q*err_x + u_fp(:,k)'*R*u_fp(:,k));
end

% Add terminal cost   
Q = fc.Q(:,:,Q_key(wp_fw,2));

err_x = x_fp(:,N)-x_targ;
cost_curr = cost_curr + 0.5* err_x'*Q*err_x;

disp(['[ilq_fp]: Current Cost: ',num2str(cost_curr)]);

% If cost goes down, we know it's feasible. Update x_bar.
u_diff = sum(vecnorm(u_bar-u_fp));
x_bar = x_fp;
u_bar = u_fp;

end