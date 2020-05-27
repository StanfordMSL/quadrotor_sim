function [l,L] = al_ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,A_cons,b_cons,mu,lambda)

v = Q_f*(x_bar(:,end)-x_itr(:,end));
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % Generate constraint partials
    c_con   = -A_cons*u_bar(:,k) + b_cons;
    c_con_u = -A_cons;
    
    I_mu = zeros(4,4);
    for j = 1:4
        if c_con(j,1) < 0 && lambda(j,k) == 0
            % Carry on
        else
            I_mu(j,j) = mu(j,k);
        end
    end
    
    % Update the Stagewise Variables
    c_x  = Q_t *(x_bar(:,k)-x_itr(:,k));
    c_u  = R*u_bar(:,k);
    c_xx = Q_t;
    c_uu = R;
    c_ux = zeros(4,13);
    
    Q_x  = c_x  + A(:,:,k)'*v;
    Q_u  = c_u  + B(:,:,k)'*v + c_con_u'*(lambda(:,k) + I_mu*c_con);
    Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k);
    Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k) + c_con_u'*I_mu*c_con_u;
    Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k);
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end