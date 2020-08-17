function [l_itr,L_itr] = backward_pass(x_bar,u_bar,obj_s,model,wts,al)     
    %% Unpack and define some useful stuff
    
    % Weights
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd; 
    
    % Some Useful Counts
    N_bp = size(x_bar,2);
    
    % Objectives
    x_star = obj_s.x_star;
    u_star = model.race_u;
    
    %% Generate the necessary linear terms
    [A,B] = dynamics_linearizer(x_bar,u_bar,model);
    
    %% Generate the constraint
    lambda = al.lambda;
    
    %% Execute the Backward Pass
    
    % Initial Step    
    v = Q_f*(x_bar(:,end)-x_star);
    V = Q_f;

    for k = N_bp-1:-1:1
        % Extract constraint partials
        I_mu  = al.I_mu(:,:,k);
        con   = al.con(:,k);
        con_x = al.con_x(:,:,k);
        con_u = al.con_u(:,:,k);
        
        % Update the Stagewise Variables
        c_x  = Q_t *(x_bar(:,k)-x_star);
        c_u  = R*(u_bar(:,k)-u_star);
        c_xx = Q_t;
        c_uu = R;
        c_ux = zeros(4,13);

        % Update intermediate variables
        Q_x  = c_x  + A(:,:,k)'*v + con_x'*(lambda(:,k) + I_mu*con);
        Q_u  = c_u  + B(:,:,k)'*v + con_u'*(lambda(:,k) + I_mu*con);
        Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k) + con_x'*I_mu*con_x;
        Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k) + con_u'*I_mu*con_u;
        Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k) + con_u'*I_mu*con_x;

        % Update the feed-forward and feedback terms
        l_itr(:,:,k) = -(Q_uu+0.01.*model.rho.*eye(4))\Q_u;
        L_itr(:,:,k) = -(Q_uu+0.01.*model.rho.*eye(4))\Q_ux;
%         l_itr(:,:,k) = -Q_uu\Q_u;
%         L_itr(:,:,k) = -Q_uu\Q_ux;
        
        % Update v and V for next bp state
        v = Q_x - L_itr(:,:,k)'*Q_uu*l_itr(:,k);
        V = Q_xx - L_itr(:,:,k)'*Q_uu*L_itr(:,:,k);
    end
    
end