function [l,L,delV] = backward_pass(X,U,con,lambda,mu_diag,xs,us)

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
N   = size(X,2);

% Generate the nominals and ideals (Xstar, Ustar)
Xs = [X(:,2:end) xs];
Us = repmat(us,1,N-1);

% Tuning Parameter
rho = 0.001;
reg = zeros(n_u,n_u);

% Initialize feedback policy variables
l = zeros(n_u,N-1);
L = zeros(n_u,n_x,N-1);
delV = zeros(2,N);

% Initial
Q_N = Q_N_calc(1);
q_N = q_N_calc(X(:,N),Xs(:,N));

V = Q_N;
v = q_N;

for k = N-1:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);
    
    xs = Xs(:,k);
    us = Us(:,k);
    
    A = A_calc(x,u);
    B = B_calc(x,u);
    
    c  = con.c(:,k);
    cx = con.cx(:,:,k);
    cu = con.cu(:,:,k);
    
    ld = lambda(:,k);
    I_mu = diag(mu_diag(:,k));
    
    % Generate Intermediate Terms
    Q_k = Q_k_calc(1);
    R_k = R_k_calc(1);
    q_k = q_k_calc(x,xs);
    r_k = r_k_calc(u,us);
        
    Qx  = q_k + A'*v + cx'*(ld + I_mu*c);
    Qu  = r_k + B'*v + cu'*(ld + I_mu*c);
    Qxx = Q_k + A'*V*A + cx'*I_mu*cx;
    
    Quu = R_k + B'*V*B + cu'*I_mu*cu + reg;
    while rcond(Quu) < 1e-5
        reg = rho.*eye(n_u);
        rho = 10.*rho;
        
        Quu = R_k + B'*V*B + cu'*I_mu*cu + reg;
%         disp(['[backward_pass]: reg increased to ',num2str(rho)]);
    end
    Qux = B'*V*A + cu'*I_mu*cx;
    
%     Quuh = R_k + B'*(V+reg)*B + cu'*I_mu*cu;
%     
%     % Regularize the term to be inverted
%     while rcond(Quuh) < 1e-5
%         reg = 10.*reg;
%         Quuh = R_k + B'*(V+reg)*B + cu'*I_mu*cu;
%     end
%     Quxh = B'*(V+reg)*A + cu'*I_mu*cx;
    
    % Generate Feedback Update
    l(:,k)   = -Quu\Qu;
    L(:,:,k) = -Quu\Qux;
    
    % Generate next cost-to-go
    V = Qxx + L(:,:,k)'*Quu*L(:,:,k) + L(:,:,k)'*Qux + Qux'*L(:,:,k);
    v = Qx  + L(:,:,k)'*Quu*l(:,k) + L(:,:,k)'*Qu + Qux'*l(:,k) ;
    
    % Generate line-search checker
    delV(1,k) = (l(:,k)' * Qu);
    delV(2,k) = 0.5.*(l(:,k)' * Quu * l(:,k));
end