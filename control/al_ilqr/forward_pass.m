function [X,U,con,La_c] = forward_pass(X,U,l,L,delV,La_p,lambda,mu,xs,us)

% Tuning Parameter
alpha = diag([10,1,1,1]);

% Unpack the relevant variables
Xb = X;
Ub = U;

N = size(X,2);
n_x = size(X,1);
n_u = size(U,1);

Xact = zeros(13,N);
Xact(:,1) = [X(:,1) ; zeros(3,1)];

Xfp = zeros(n_x,N);
Xfp(:,1) = X(:,1);
Ufp = zeros(n_u,N-1);

% Forward pass assumes no external forces and noise
FT_ext = zeros(6,1);  
wt = zeros(13,1);

while true
    br = br_init(); 

    for k = 1:N-1
        del_x = Xact(1:10,k) - Xb(:,k);

        u_op = Ub(:,k) + alpha*l(:,k);
        u_cl = L(:,:,k)*del_x;
        
        Ufp(:,k) = u_op +  u_cl;
        Ufp(:,k) = u_op;

        [u_wr,br] = br_ctrl(Xact(:,k),Ufp(:,k),br);
        u_mt = w2m_est(u_wr);
        
        Xact(:,k+1) = quadcopter_est(Xact(:,k),u_mt,FT_ext,wt);
        Xfp(:,k+1)  = Xact(1:10,k+1);
    end
    
    con = con_calc(Xfp,Ufp);
    
    mu_diag = check_con(con.c,lambda,mu);
    La_c = lagr_calc(Xfp,Ufp,xs,us,con.c,lambda,mu_diag);

    % Debug
%     La_plot(La_p,La_c);
%     disp(['[forward_pass]: alpha = ',num2str(alpha)]);

    [flag_LS,alpha] = check_LS(La_c,La_p,alpha,delV);
    if flag_LS == 0     
        % Constraint Cost Improved. Allow a Trajectory Update
        X = Xfp;
        U = Ufp;
        break;
    elseif flag_LS == 1
        % Try simply reducing alpha.
    elseif flag_LS == 2
        % Reducing alpha not helping. Resorting to alternatives.
        break
    end
end