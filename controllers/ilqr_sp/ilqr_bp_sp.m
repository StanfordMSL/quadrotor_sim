function [l,L,c_con] = ilqr_bp_sp(x_star,x_bar,u_bar,A,B,wts,mu,lambda,model,n_con)

    % Unpack Stuff
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;

    N = size(x_bar,2);

    % Initial Step    
    v = Q_f*(x_bar(:,end)-x_star);
    V = Q_f;

    % Prep the cost constraint variables
    c_con = zeros(n_con,N-1);

    % Execute the Backward Pass
    for k = N-1:-1:1
        % Generate constraint partials
        c_con(:,k) = [u_bar(5,k)-model.dt_ctl_max ; -u_bar(5,k)+model.dt_ctl_min];
        c_con_x = zeros(2,14);
        c_con_u = zeros(2,5);

        I_mu = zeros(n_con,n_con);
        for j = 1:n_con
            if c_con(j,k) <= 0 && lambda(j,k) == 0
                % Constraint not violated. Carry on.
            else
                % Constraint violated. Turn on Augment.
                I_mu(j,j) = mu(j,k);
            end
        end

        % Update the Stagewise Variables
        c_x  = Q_t *(x_bar(:,k)-x_star);
        c_u  = R*u_bar(:,k);
        c_xx = Q_t;
        c_uu = R;
        c_ux = zeros(5,14);

        % Update intermediate variables
        Q_x  = c_x  + A(:,:,k)'*v + c_con_x'*(lambda(:,k) + I_mu*c_con(:,k));
        Q_u  = c_u  + B(:,:,k)'*v + c_con_u'*(lambda(:,k) + I_mu*c_con(:,k));
        Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k) + c_con_x'*I_mu*c_con_x;
        Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k) + c_con_u'*I_mu*c_con_u;
        Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k) + c_con_u'*I_mu*c_con_x;

        % Update the feed-forward and feedback terms
        l(:,:,k) = -(Q_uu+model.rho.*eye(5))\Q_u;
        L(:,:,k) = -(Q_uu+model.rho.*eye(5))\Q_ux;

        % Update v and V for next bp state
        v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
        V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
    end

end