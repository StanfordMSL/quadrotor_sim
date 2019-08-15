function [l,L] = ilqr_bp(t_bar,x_bar,u_bar,wp,A,B,N,fc)

% Unpack Waypoint Terms
t_wp = wp.t;
x_wp = wp.x;
Q_key = wp.Q_key;

% Initialize BP
R = fc.R;

wp_bk = wp.N_wp;
Q =  fc.Q(:,:,Q_key(wp_bk,2));
x_targ = x_wp(:,wp_bk+1);

v = Q*(x_bar(:,N)-x_targ);
V = Q;

% Execute the Backward Pass
for k = N-1:-1:1
    t_bp = t_bar(k);

    % Update Q and R according to the waypoints
    if (wp_bk > 2) && (t_bp < t_wp(wp_bk))
        wp_bk = wp_bk-1;
        Q = fc.Q(:,:,Q_key(wp_bk,2));
        x_targ = x_wp(:,wp_bk+1);
    else
        Q = fc.Q(:,:,Q_key(wp_bk,1));
    end 

    % Update the Stagewise Variables
    Q_x  = Q *(x_bar(:,k)-x_targ) + A(:,:,k)'*v;
    Q_xx = Q + A(:,:,k)'*V*A(:,:,k);
    Q_u  = R*u_bar(:,k) + B(:,:,k)'*v;
    Q_uu = R + B(:,:,k)'*V*B(:,:,k);
    Q_ux = B(:,:,k)'*V*A(:,:,k);
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end