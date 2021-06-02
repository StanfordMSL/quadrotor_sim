function [l,L,delV] = backward_pass(X,U,con,con_x,con_u,lambda,mu_diag)

% Rearrange Some Stuff
U = [U(1,:) ; X(11:13,1:end-1)];
X = X(1:10,:);

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
N   = size(X,2);

% Tuning Parameter
reg_init = zeros(n_x,n_x);
reg = reg_init;

% Initialize feedback policy variables
l = zeros(n_u,N-1);
L = zeros(n_u,n_x,N-1);
delV = zeros(2,N);

% Initial
Q_N = Q_N_calc(1);
q_N = q_N_calc(X(:,N));

V = Q_N;
v = q_N;

for k = N-1:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);
    
    A = A_calc(x,u);
    B = B_calc(x,u);
    
    c = con(:,k);
    cx = con_x(:,:,k);
    cu = con_u(:,:,k);
    
    ld = lambda(:,k);
    I_mu = diag(mu_diag(:,k));
    
    % Generate Intermediate Terms
    Q_k = Q_k_calc(1);
    R_k = R_k_calc(1);
    q_k = q_k_calc(1);
    r_k = r_k_calc(1);
        
    Qx  = q_k + A'*v + cx'*(ld + I_mu*c);
    Qu  = r_k + B'*v + cu'*(ld + I_mu*c);
    Qxx = Q_k + A'*V*A + cx'*I_mu*cx;
    
    Quu = R_k + B'*V*B + cu'*I_mu*cu;
    Qux = B'*V*A + cu'*I_mu*cx;
    
    Quuh = R_k + B'*(V+reg)*B + cu'*I_mu*cu;
    
    % Regularize the term to be inverted
    while rcond(Quuh) < 1e-5
        reg = reg + eye(n_x);
        Quuh = R_k + B'*(V+reg)*B + cu'*I_mu*cu;
    end
    Quxh = B'*(V+reg)*A + cu'*I_mu*cx;
    reg = reg_init;
    
    % Generate Feedback Update
    l(:,k)   = -Quuh\Qu;
    L(:,:,k) = -Quuh\Quxh;
    
    % Generate next cost-to-go
    V = Qxx + L(:,:,k)'*Quu*L(:,:,k) + L(:,:,k)'*Qux + Qux'*L(:,:,k);
    v = Qx  + L(:,:,k)'*Quu*l(:,k) + L(:,:,k)'*Qu + Qux'*l(:,k) ;
    
    % Generate line-search checker
    delV(1,k) = (l(:,k)' * Qu);
    delV(2,k) = 0.5.*(l(:,k)' * Quu * l(:,k));
end