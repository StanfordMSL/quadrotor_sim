function [l,L] = ileqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,W_inv,gamma)

v = Q_f*(x_bar(:,end)-x_itr(:,end));
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % LEQR step
    V_tilde = V + gamma.*V*((W_inv-gamma.*V)\V);

    % Update the Stagewise Variables
    Q_x  = Q_t *(x_bar(:,k)-x_itr(:,k)) + A(:,:,k)'*v;
    Q_xx = Q_t + A(:,:,k)'*V_tilde*A(:,:,k);
    Q_u  = R*u_bar(:,k) + B(:,:,k)'*v;
    Q_uu = R + B(:,:,k)'*V_tilde*B(:,:,k);
    Q_ux = B(:,:,k)'*V_tilde*A(:,:,k);
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+eye(4))\Q_ux;

%     l_test = sum(isnan(l(:,:,k)));
%     L_test = sum(isnan(l(:,:,k)),1:2);
%     if (l_test > 0) || (L_test >0)
%         disp('[ilqr_bp]: NaN detected in l and/or L.')
%     end

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end