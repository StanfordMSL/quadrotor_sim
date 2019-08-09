function [l,L] = ilqr_bp(x_bar,u_bar,t_now,wp,A,B,N,model,fc)

% Unpack some terms
t_wp = wp.t;
x_wp = wp.x;

Q_key = wp.Q_key;
Q     = fc.Q;
R     = fc.R;
rho   = fc.rho;
        
% Initialize v and V
wp_bk = wp.N_wp;

v = Q(:,:,Q_key(wp_bk))*(x_bar(:,N)-x_wp(:,wp_bk));
V = Q(:,:,Q_key(wp_bk));

wp_bk = wp_bk-1;

for k = N-1:-1:1
    t_bp = k*model.fc_dt+t_now;

    % Determine Some Useful Terms
    if (wp_bk > 0) && (abs(t_bp - t_wp(wp_bk)) <= 1e-3)
        Q_x  = Q(:,:,Q_key(wp_bk)) *(x_bar(:,k)-x_wp(:,wp_bk)) + A(:,:,k)'*v;
        Q_xx = Q(:,:,Q_key(wp_bk)) + A(:,:,k)'*V*A(:,:,k);
    
        wp_bk = wp_bk - 1;
        
    else
        Q_x  = Q(:,:,1)*x_bar(:,k) + A(:,:,k)'*v;
        Q_xx = Q(:,:,1) + A(:,:,k)'*V*A(:,:,k);
    end 

    Q_u  = R*u_bar(:,k) + B(:,:,k)'*v;
    Q_uu = R + B(:,:,k)'*V*B(:,:,k);
    Q_ux = B(:,:,k)'*V*A(:,:,k);

    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+rho*eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+rho*eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end