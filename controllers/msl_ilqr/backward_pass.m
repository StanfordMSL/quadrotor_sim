function [l_itr,L_itr,del_V] = backward_pass(x_bar,u_bar,al,rho,cost_param,model)     
    %% Unpack and initialize some useful stuff
    % State and input parameters
    x_star = cost_param.x_star;
    u_star = cost_param.u_star;
    n_x = size(x_star,1);
    n_u = size(u_star,1);
        
    % Counts
    N_bp = size(x_bar,2);
    
    % Initialize feedback policy variables
    l_itr = zeros(n_u,N_bp-1);
    L_itr = zeros(n_u,n_x,N_bp-1);
    del_V = zeros(2,N_bp);
    
    %% Execute the Backward Pass
    % Initial Step
    x_bp  = x_bar(:,end);
    Q     = cost_param.Q(:,:,end);
    v     = Q*(x_bp-x_star);
    V     = Q;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     K = 1e-9;
%     q_bp  = x_bp(7:10,1);
%     q_star = cost_param.x_star(7:10,1);
% 
%     q_dot = q_star'*q_bp;
%     q_dif = 1-q_dot^2;
%     delta = acos(q_dot);
%     
%     if delta == 0
%         c_v = 0;
%         c_V = 0;
%     else
%         c_dq = -1/sqrt(q_dif);
%         
%         c_v  =  4*(delta^3)*c_dq;
%         c_V  = (-4*(delta^3)/q_dif) * ( (3/delta) + q_dot/sqrt(q_dif) );
%     end
%     
%     q_geo = c_v .*  q_star;
%     Q_geo = c_V .* (q_star*q_star');
%     
%     q_feed = zeros(13,1);
%     Q_feed = zeros(13,13);
%     
%     q_feed(7:10,1) = q_geo;
%     Q_feed(7:10,7:10) = Q_geo;
%     
%     v     = v + K.*q_feed;  
%     V     = V + K.*Q_feed;
%     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for k = N_bp-1:-1:1
        % Extract states and inputs
        x_bp = x_bar(:,k);
        u_bp = u_bar(:,k);
        
        % Extract cost weights
        Q     = cost_param.Q(:,:,k);
        R     = cost_param.R(:,:,k);
    
        % Generate linearization
        [A,B] = stagewise_linearizer(x_bp,u_bp,model);
    
        % Extract constraint variables
        lambda = al.lambda(:,k);
        I_mu   = al.I_mu(:,:,k);
        con    = al.con(:,k);
        con_x  = al.con_x(:,:,k);
        con_u  = al.con_u(:,:,k);
        
        % Update the Stagewise Variables
        c_x  = Q * (x_bp-x_star);
        c_u  = R * (u_bp-u_star);
        c_xx = Q;
        c_uu = R;
        c_ux = zeros(4,13);

        % Update intermediate variables
        Q_x  = c_x  + A'*v + con_x'*(lambda + I_mu*con);
        Q_u  = c_u  + B'*v + con_u'*(lambda + I_mu*con);
        Q_xx = c_xx + A'*V*A + con_x'*I_mu*con_x;
        Q_uu = c_uu + B'*V*B + con_u'*I_mu*con_u;
        Q_ux = c_ux + B'*V*A + con_u'*I_mu*con_x;
        
        % Update the feed-forward and feedback terms
        l_itr(:,k)   = -(Q_uu+rho.*eye(4))\Q_u;
        L_itr(:,:,k) = -(Q_uu+rho.*eye(4))\Q_ux;
        
        % Update v and V for next bp state
        v = Q_x  - L_itr(:,:,k)'*Q_uu*l_itr(:,k);
        V = Q_xx - L_itr(:,:,k)'*Q_uu*L_itr(:,:,k);
        
        % Expected change in cost
        del_V(1,k) = (l_itr(:,k)' * Q_u);
        del_V(2,k) = 0.5.*(l_itr(:,k)' * Q_uu * l_itr(:,k));
    end
    
end