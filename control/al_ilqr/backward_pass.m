function [l,L,delV] = backward_pass(X,U,lqr,con,mult,mode)

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
N   = size(X,2);
xs  = lqr.xs;
us  = lqr.us;
T   = lqr.T;

% Tuning Parameter
rho = 0.001;
reg = zeros(n_u,n_u);

% Initialize feedback policy variables
l = zeros(n_u,N-1);
L = zeros(n_u,n_x,N-1);
delV = zeros(2,N);

% Initial
V = dQN(1);
v = dqN(X(:,N),xs,1);

for k = N-1:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);

    A = A_calc(x,u);
    B = B_calc(x,u);
    
    c  = con.c(:,k);
    cx = con.cx(:,:,k);
    cu = con.cu(:,:,k);
    
    lamx = mult.lamx(:,k);
    lamu = mult.lamu(:,k);
    mudx = mult.mudx(:,k);
    mudu = mult.mudu(:,k);
    
    lam = [ lamx ; lamu];
    I_mu = diag([ mudx ; mudu ] );
    
    % Generate Intermediate Terms
    if k < T
        dQk = dQn(1);
        dRk = dRn(1);
        dqk = dqn(x,xs,1);
        drk = drn(u,us,1);
    else
        dQk = dQT(1);
        dRk = dRT(1);
        dqk = dqT(x,xs,1);
        drk = drT(u,us,1);
    end
        
    Qx  = dqk + A'*v + cx'*(lam + I_mu*c);
    Qu  = drk + B'*v + cu'*(lam + I_mu*c);
    Qxx = dQk + A'*V*A + cx'*I_mu*cx;
    
    switch mode
        case 'fast'
            % Generate Feedback Update
            l(:,k)   = -Qu;
        case 'slow'
            Quu = dRk + B'*V*B + cu'*I_mu*cu + reg;
            while rcond(Quu) < 1e-5
                reg = rho.*eye(n_u);
                rho = 10.*rho;
                
                Quu = dRk + B'*V*B + cu'*I_mu*cu + reg;
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
    end
    
    % Generate next cost-to-go
    V = Qxx + L(:,:,k)'*Quu*L(:,:,k) + L(:,:,k)'*Qux + Qux'*L(:,:,k);
    v = Qx  + L(:,:,k)'*Quu*l(:,k) + L(:,:,k)'*Qu + Qux'*l(:,k) ;
    
    % Generate line-search checker
    delV(1,k) = (l(:,k)' * Qu);
    delV(2,k) = 0.5.*(l(:,k)' * Quu * l(:,k));
end