function [l,L,c_con] = al_ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,mu,lambda,model,wp,n_con)

v = Q_f*(x_bar(:,end)-x_itr(:,end));
V = Q_f;

N = size(x_bar,2);

% Prep the cost constraint variables
c_con = zeros(n_con,N-1);

% Execute the Backward Pass
for k = N-1:-1:1
    % Generate constraint partials
    [c_con(:,k), c_con_x, c_con_u] = compute_ineq_v2(x_bar(:,k),u_bar(:,k),model,wp,n_con);
    
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
    c_x  = Q_t *(x_bar(:,k)-x_itr(:,k));
    c_u  = R*u_bar(:,k);
    c_xx = Q_t;
    c_uu = R;
    c_ux = zeros(4,13);
    
    Q_x  = c_x  + A(:,:,k)'*v + c_con_x'*(lambda(:,k) + I_mu*c_con(:,k));
    Q_u  = c_u  + B(:,:,k)'*v + c_con_u'*(lambda(:,k) + I_mu*c_con(:,k));
    Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k) + c_con_x'*I_mu*c_con_x;
    Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k) + c_con_u'*I_mu*c_con_u;
    Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k) + c_con_u'*I_mu*c_con_x;
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+model.rho.*eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+model.rho.*eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end