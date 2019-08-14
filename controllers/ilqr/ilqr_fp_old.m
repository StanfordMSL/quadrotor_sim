function [x_bar,u_bar,u_diff,alpha] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,model,fc)

% Unpack some terms
t_wp = wp.t;
x_wp = wp.x;
Q_key = wp.Q_key;

% Initialize some terms
itrs = 1;
alpha_test = 1;

% We try to find the ideal alpha.
while 1
    % Reset Forward Pass Variables.
    x_fp = zeros(12,N);
    x_fp(:,1) = x_now;
    u_fp = u_bar; 

    % Initialize FP
    wp_k = 2;
    Q = fc.Q(:,:,Q_key(wp_k));
    R = fc.R;
    x_targ = x_wp(:,wp_k);

    cost_curr =  0;
    
    for k = 1:N-1
        t_fp = t_bar(k);
        
        % Determine Control Command
        del_x = x_fp(:,k)-x_bar(:,k);
        del_u = alpha_test*l(:,:,k) + L(:,:,k)*del_x;
        u_fp(:,k) = u_fp(:,k) + del_u;
             
        % Predict Dynamics of Next Step
        FT_ext = zeros(6,1);
        m_cmd = wrench2m_cmd(u_fp(:,k),model);

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
    err_x = x_fp(:,N)-x_targ;
    cost_curr = cost_curr + 0.5* err_x'*Q*err_x;
    
    fast_plot(x_fp);
    if itrs == 1
        x_first = x_fp;
        u_first = u_fp;

        cost_first = cost_curr;

        disp(['[ilqr_fp]: Initial Cost Calculation: ',num2str(cost_curr)]);
        
        itrs = itrs + 1;
    elseif (cost_first > cost_curr) && itrs > 1
        disp(['[ilqr_fp]: Lower alpha found. Cost: ',num2str(cost_curr),'  Alpha: ',num2str(alpha_test)]);
        break;
    elseif alpha_test == 1
        x_fp = x_first;
        u_fp = u_first;
        alpha_test = 1;

        disp('ilqr_fp: Alpha below threshold. Reverting to alpha = 1.');
        break;
    else
        alpha_test = 0.5*alpha_test;

        disp(['[ilqr_fp]: Current Cost: ',num2str(cost_curr),...
             ' [ilqr_fp]: Iterations: ',num2str(itrs),'  Alpha: ',num2str(alpha_test)]);

        itrs = itrs + 1;
    end
    
    fast_plot(x_fp);
end

% If cost goes down, we know it's feasible. Update x_bar.
u_diff = sum(vecnorm(u_bar-u_fp));
x_bar = x_fp;
u_bar = u_fp;
alpha = alpha_test;

end