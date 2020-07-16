function [l,L] = ilqr_bp_sp(x_star,x_bar,u_bar,A,B,wts,model,al)

    % Unpack Stuff
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;

    N_al = size(x_bar,2);

    % Initial Step    
    v = Q_f*(x_bar(:,end)-x_star);
    V = Q_f;

    % Execute the Backward Pass
    for k = N_al-1:-1:1
        % Generate constraint terms

        u_curr  = u_bar(:,k);
        ld_curr = al.lambda(:,k);
        mu_curr = al.mu(:,k);
        if k == 1
            u_prev  = u_bar(:,k+1);
        else
            u_prev = u_bar(:,k-1);
        end
        [I_mu,c_con,c_con_x,c_con_u] = con_builder(u_curr,u_prev,ld_curr,mu_curr,model);
        
        % Update the Stagewise Variables
        c_x  = Q_t *(x_bar(:,k)-x_star);
        c_u  = R*u_bar(:,k);
        c_xx = Q_t;
        c_uu = R;
        c_ux = zeros(5,13);

        % Update intermediate variables
        Q_x  = c_x  + A(:,:,k)'*v + c_con_x'*(ld_curr + I_mu*c_con);
        Q_u  = c_u  + B(:,:,k)'*v + c_con_u'*(ld_curr + I_mu*c_con);
        Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k) + c_con_x'*I_mu*c_con_x;
        Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k) + c_con_u'*I_mu*c_con_u;
        Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k) + c_con_u'*I_mu*c_con_x;

        % Update the feed-forward and feedback terms
%         l(:,:,k) = -(Q_uu+model.rho.*eye(5))\Q_u;
%         L(:,:,k) = -(Q_uu+model.rho.*eye(5))\Q_ux;
        l(:,:,k) = -Q_uu\Q_u;
        L(:,:,k) = -Q_uu\Q_ux;
        
        % Update v and V for next bp state
        v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
        V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
    end
%     temp = zeros(1,1000);
%     for j = 1:1000
%         temp(1,j) = l(5,:,j);
%     end
end