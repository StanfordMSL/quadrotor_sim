function [Xfp,Ufp,Xact,Umt] = forward_pass(Xbar,Ubar,l,L,alpha)

% Unpack the relevant variables
N = size(Xbar,2);
n_x = size(Xbar,1);
n_u = size(Ubar,1);

% Forward Pass Dynamics have no noise
FT_ext = zeros(6,1);  
wt = zeros(13,1);

% Prepare Container Variables
Xact = zeros(13,N);
Xact(1:10,1) = Xbar(1:10,1);

Z = zeros(7,N);

Xfp = zeros(n_x,N);
Xfp(:,1) = Xbar(:,1);
Ufp = zeros(n_u,N-1);

Umt = zeros(n_u,N-1);

% Roll the Dynamic Forward (to get candidates: u_fp,x_fp)
br = br_init(); 
for k = 1:N-1
    x_k = Xact(:,k);
%     z_k = Z(:,k);
%     p_k = Xact(1:3,k);
%     q_k = Xact(7:10,k);
%     p_bar = Xbar(1:3,k);
%     q_bar = Xbar(7:10,k);
    
    del_x = Xfp(:,k)-Xbar(:,k);
    
    u_ol = alpha*l(:,k);
    u_cl = L(:,:,k)*del_x;
    u_fp = Ubar(:,k) + u_ol + u_cl;

    [u_wr,br] = br_ctrl(x_k,u_fp,br);
    u_mt = w2m(u_wr);

    Xact(:,k+1) = quadcopter_est(x_k,u_mt,FT_ext,wt);
    Xfp(1:10,k+1) = Xact(1:10,k+1);
    
%     Z(:,k+1)= error_upd(z_k,p_k,q_k,p_bar,q_bar);
%     Xfp(11:17,k+1) = Z(:,k+1);
    
    Ufp(:,k)    = u_fp;
    Umt(:,k)    = u_mt;
end
% figure(1)
% clf
% plot(Xact(11,1:k))
% hold on
% plot(Ubar(2,1:k));
% ylim([-5 5]);
end