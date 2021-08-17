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
Xact(1:10,1) = Xbar(:,1);

Xfp = zeros(n_x,N);
Xfp(:,1) = Xbar(:,1);
Ufp = zeros(n_u,N-1);

Umt = zeros(n_u,N-1);

% Roll the Dynamic Forward (to get candidates: u_fp,x_fp)
br = br_init(); 
for k = 1:N-1
    x_now = Xact(:,k);
    del_x = Xfp(:,k)-Xbar(:,k);
    
    u_ol = alpha*l(:,k);
    u_cl = L(:,:,k)*del_x;
    u_fp = Ubar(:,k) + u_ol + u_cl;

    [u_wr,br] = br_ctrl(x_now,u_fp,br);
    u_mt = w2m_est(u_wr);

    Xact(:,k+1) = quadcopter_est(x_now,u_mt,FT_ext,wt);
    Xfp(:,k+1)  = Xact(1:10,k+1);
    Ufp(:,k)    = u_fp;
    Umt(:,k)    = u_mt;
end

end