function cdd = backward_pass(x_bar,u_bar,al,rho,cost_param)     
    %% Unpack and initialize some useful stuff
    % State and input parameters
    n_x = size(x_bar,1);
    n_u = size(u_bar,1);
        
    % Counts
    N_bp = size(x_bar,2);
    
    % Initialize feedback policy variables
    l = zeros(n_u,N_bp-1);
    L = zeros(n_u,n_x,N_bp-1);
    del_V = zeros(2,N_bp);
    
    %% Execute the Backward Pass
    % Initial Step
    x_bp  = x_bar(:,end);
    x_N   = cost_param.x_N;
    
    Q     = cost_param.Q_N;
    
    v     = Q*(x_bp-x_N);
    V     = Q;
    
    for k = N_bp-1:-1:1
        % Extract states and inputs
        x_bp = x_bar(:,k);
        u_bp = u_bar(:,k);
        x_k = cost_param.x_k(:,k);
        u_k = cost_param.u_k(:,k);

        % Extract cost weights
        Q     = cost_param.Q_k(:,:,k);
        R     = cost_param.R_k(:,:,k);
    
        % Generate linearization
        A = A_calc(x_bp,u_bp);
        B = B_calc(x_bp,u_bp);
    
        % Extract constraint variables
        lambda = al.lambda(:,k);
        I_mu   = al.I_mu(:,:,k);
        con    = al.con(:,k);
        con_x  = al.con_x(:,:,k);
        con_u  = al.con_u(:,:,k);
        
        % Update the Stagewise Variables
        c_x  = Q * (x_bp-x_k);
        c_u  = R * (u_bp-u_k);
        c_xx = Q;
        c_uu = R;
        c_ux = zeros(4,n_x);

        % Update intermediate variables
        Q_x  = c_x  + A'*v + con_x'*(lambda + I_mu*con);
        Q_u  = c_u  + B'*v + con_u'*(lambda + I_mu*con);
        Q_xx = c_xx + A'*V*A + con_x'*I_mu*con_x;
        Q_uu = c_uu + B'*V*B + con_u'*I_mu*con_u;
        Q_ux = c_ux + B'*V*A + con_u'*I_mu*con_x;
        
        % Update the feed-forward and feedback terms
        l(:,k)   = -(Q_uu+rho.*eye(4))\Q_u;
        L(:,:,k) = -(Q_uu+rho.*eye(4))\Q_ux;
        
        % Update v and V for next bp state
        v = Q_x  - L(:,:,k)'*Q_uu*l(:,k);
        V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
        
        % Expected change in cost
        del_V(1,k) = (l(:,k)' * Q_u);
        del_V(2,k) = 0.5.*(l(:,k)' * Q_uu * l(:,k));
    end
    cdd.l = l;
    cdd.L = L;
    cdd.del_V = del_V;
end