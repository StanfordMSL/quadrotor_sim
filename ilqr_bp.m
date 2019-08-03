function [l,L] = ilqr_bp(x_bar,u_bar,x_wp,A,B,N,fc)

% Unpack some terms
Q_lqr = fc.Q;
R_lqr = fc.R;
Q_N   = fc.Q_N;
rho   = fc.rho;
        
% Initialize v and V
v = Q_N*(x_bar(:,end)-x_wp);
V = Q_N;

for k = N:-1:1
    % Determine Some Useful Terms
    Q_x  = Q_lqr*x_bar(:,k) + A(:,:,k)'*v;
    Q_u  = R_lqr*u_bar(:,k) + B(:,:,k)'*v;
    Q_xx = Q_lqr + A(:,:,k)'*V*A(:,:,k);
    Q_uu = R_lqr + B(:,:,k)'*V*B(:,:,k);
    Q_ux = B(:,:,k)'*V*A(:,:,k);

    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+rho*eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+rho*eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end