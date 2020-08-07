function [l,L] = ileqr_oa_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,W_inv,gamma,coeff_obs)

v = Q_f*(x_bar(:,end)-x_itr(:,end));
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % LEQR step
    V_tilde = V + gamma.*V*((W_inv-gamma.*V)\V);

    % Update the Stagewise Variables
    [dc_x1, dc_xx1] = obs_cost(x_bar(:,k),coeff_obs(1,1),coeff_obs(1,2),[0 0 1.1]');
    [dc_x2, dc_xx2] = obs_cost(x_bar(:,k),coeff_obs(2,1),coeff_obs(2,2),[0 0 1.0]');
    c_x  = Q_t *(x_bar(:,k)-x_itr(:,k)) + dc_x1 + dc_x2;
    c_xx = Q_t + dc_xx1 + dc_xx2;

%     [dc_x1, dc_xx1] = obs_cost(x_bar(:,k),coeff_obs(1,1),coeff_obs(1,2),[0 0 1]');
%     c_x  = Q_t *(x_bar(:,k)-x_itr(:,k)) + dc_x1;
%     c_xx = Q_t + dc_xx1;
    
    c_u  = R*u_bar(:,k);
    c_uu = R;
    c_ux = zeros(4,13);
    
    Q_x  = c_x  + A(:,:,k)'*v;
    Q_u  = c_u  + B(:,:,k)'*v;
    Q_xx = c_xx + A(:,:,k)'*V_tilde*A(:,:,k);
    Q_uu = c_uu + B(:,:,k)'*V_tilde*B(:,:,k);
    Q_ux = c_ux + B(:,:,k)'*V_tilde*A(:,:,k);
    
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