function nom = toc_ilqr_bp(x_star,nom,A,B,wts,model)

    % Unpack Stuff
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    N_toc  = nom.N_toc;
    x_bar  = nom.x_bar;
    u_bar  = nom.u_bar;   

    % Initial Step    
    v = Q_f*(x_bar(:,end)-x_star);
    V = Q_f;

    % Execute the Backward Pass
    for k = N_toc-1:-1:1
        % Update the Stagewise Variables
        c_x  = Q_t *(x_bar(:,k)-x_star);
        c_u  = R*u_bar(:,k);
        c_xx = Q_t;
        c_uu = R;
        c_ux = zeros(4,13);

        % Update intermediate variables
        Q_x  = c_x  + A(:,:,k)'*v;
        Q_u  = c_u  + B(:,:,k)'*v;
        Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k);
        Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k);
        Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k);

        % Update the feed-forward and feedback terms
        l(:,:,k) = -(Q_uu+model.rho.*eye(4))\Q_u;
        L(:,:,k) = -(Q_uu+model.rho.*eye(4))\Q_ux;
%         l(:,:,k) = -Q_uu\Q_u;
%         L(:,:,k) = -Q_uu\Q_ux;
        
        % Update v and V for next bp state
        v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
        V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
    end
    
    nom.l = l;
    nom.L = L;
end