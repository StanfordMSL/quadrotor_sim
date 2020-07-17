function [l,L] = t_ilqr_bp(x_star,x_bar,u_bar,A,B,Q_t,Q_f,R)

v = Q_f*(x_bar(:,end)-x_star);
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % Update the Stagewise Variables
    c_x  = Q_t *(x_bar(:,k)-x_star);
    c_u  = R*u_bar(:,k);
    c_xx = Q_t;
    c_uu = R;
    c_ux = zeros(4,13);
    
    Q_x  = c_x  + A(:,:,k)'*v;
    Q_u  = c_u  + B(:,:,k)'*v;
    Q_xx = c_xx + A(:,:,k)'*V*A(:,:,k);
    Q_uu = c_uu + B(:,:,k)'*V*B(:,:,k);
    Q_ux = c_ux + B(:,:,k)'*V*A(:,:,k);
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+1.0.*eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+1.0.*eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end