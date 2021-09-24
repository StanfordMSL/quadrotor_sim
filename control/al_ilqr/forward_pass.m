function [Xfp,Ufp] = forward_pass(Xbar,Ubar,l,L,alpha)

% Unpack the relevant variables
N = size(Xbar,2);
n_x = size(Xbar,1);
n_u = size(Ubar,1);

% Forward Pass Dynamics have no noise
F_ext = zeros(3,1);  

% Prepare Container Variables
Xfp = zeros(n_x,N);
Xfp(:,1) = Xbar(:,1);

Ufp = zeros(n_u,N-1);

% Roll the Dynamic Forward (to get candidates: u_fp,x_fp)
for k = 1:N-1
    % Quadrotor Dynamics
    x_k = Xfp(1:10,k);
    
    del_x = Xfp(:,k)-Xbar(:,k);
    
    u_ol = alpha.*[ 0.1.*l(1,k) ; l(2:4,k)];
    u_cl = L(:,:,k)*del_x;
    u_fp = Ubar(:,k) + u_ol + u_cl;

    Xfp(1:10,k+1) = quadcopter_est(x_k,u_fp,F_ext);    
    
    % Integral Dynamics
    z_k = Xfp(11:17,k);
    p_bar = Xbar(1:3,k);
    q_bar = Xbar(7:10,k);
    
    Xfp(11:17,k+1)= error_upd(z_k,x_k,p_bar,q_bar);
    
    % Updated Input
    Ufp(:,k)    = u_fp;
end
% figure(1)
% clf
% plot(Xact(11,1:k))
% hold on
% plot(Ubar(2,1:k));
% ylim([-5 5]);
end