function [l,L,delV] = backward_pass(X,U)

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
N = size(U,2);

% LQR Parameters
Xs  = X;
Us  = U;

% Tuning Parameter
rho = 0.001;
reg = zeros(n_u,n_u);

% Initialize feedback policy variables
l = zeros(n_u,N);
L = zeros(n_u,n_x,N);
delV = zeros(2,N);

% Initial
QN = diag([100 ; 100 ; 100 ; 100 ; 100 ; 100]);
Qn = diag([10 ; 10 ; 0 ; 0 ; 0 ; 0]);
Rn = zeros(2,2);

x = X(:,N+1);
xs = Xs(:,N+1);
V = QN;
v = QN*(x-xs);


for k = N:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);
    xs = Xs(:,k);
    us = Us(:,k);
    
    A = A_calc(x,u);
    B = B_calc(x,u);
    
    % Generate Intermediate Terms
    cx = Qn*(x-xs);
    cu = Rn*(u-us);
    cxx = Qn;
    cuu = Rn;
    
    Qx  = cx + A'*v;
    Qu  = cu + B'*v;
    Qxx = cxx + A'*V*A;
    
    Quu = cuu + B'*V*B + reg;
    while rcond(Quu) < 1e-1
        reg = rho.*eye(n_u);
        rho = 10.*rho;
        
        Quu = cuu + B'*V*B + reg;
    end
    Qux = B'*V*A;
    
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

end