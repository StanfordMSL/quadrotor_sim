function [l,L] = ilqr_bp_sp(x_wp,x_bar,u_bar,A,B,wts)

% Unpack Stuff
Q_t = wts.Q_pstn;
Q_f = wts.Q_unif;
R   = wts.R_stnd;
    
% Initial Step    
v = Q_f*(x_bar(:,end)-x_wp);
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % Update the Stagewise Variables
    c_x  = Q_t *(x_bar(:,k)-x_wp);
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
    l(:,:,k) = -(Q_uu+1.0.*eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+1.0.*eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end