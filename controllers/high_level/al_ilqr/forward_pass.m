function [X,U,Jc,con,con_x,con_u] = forward_pass(X,U,l,L,delV,Jp,lambda,mu,map)

% Tuning Parameter
alpha = 1;

% Unpack the relevant variables
X_bar = X(1:10,:);
U_bar = [U(1,:) ;X(11:13,1:end-1)];

N = size(X,2);
n_x = size(X,1);
n_u = size(U,1);

X_fp = zeros(n_x,N);
X_fp(:,1) = X(:,1);
U_fp = zeros(n_u,N-1);

br = br_init(); 

% Forward pass assumes no external forces and noise
FT_ext = zeros(6,1);  
wt = zeros(13,1);

while true
    for k = 1:N-1
        del_x = X_fp(1:10,k) - X_bar(:,k);

        u_op = U_bar(:,k) + alpha*l(:,k);
        u_cl = L(:,:,k)*del_x;
        
        U_fp(:,k) = u_op +  u_cl;

        [u_wr ,br] = br_ctrl(X_fp(:,k),U_fp(:,k),br);
        u_mt = w2m_est(u_wr);
        
        X_fp(:,k+1) = quadcopter_est(X_fp(:,k),u_mt,FT_ext,wt);
    end
    
    [con, con_x, con_u] = con_calc(X_fp,U_fp);
    mu_diag = check_con(con,lambda,mu);
    
    Jc = lagr_calc(X_fp,U_fp,con,lambda,mu_diag);

    nominal_plot(X_fp,map,100,'back');

    if check_LS(Jc,Jp,alpha,delV)
        X = X_fp;
        U = U_fp;
        disp(['[forward_pass]: alpha = ',num2str(alpha)]);
        break;
    else
        alpha = 0.9.*alpha;
    end
end