function [l,L,delV] = backward_pass(X,U,lqr,con,mult,mode)

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
xs  = lqr.xs;
us  = lqr.us;
N   = lqr.N;

% Tuning Parameter
rho = 0.001;
reg = zeros(n_u,n_u);

% Initialize feedback policy variables
l = zeros(n_u,N-1);
L = zeros(n_u,n_x,N-1);
delV = zeros(2,N);

% Initial
Qin = lqr.QN;
V = dQN(Qin,1);
v = dqN(X(:,N),xs,Qin,1);

for k = N-1:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);

    A = A_calc(x,u);
    B = B_calc(x,u);
    
    c   = [ con.cx(:,k) ; con.cu(:,k)];
    c_x = [ con.cx_x(:,:,k) ; con.cu_x(:,:,k)];
    c_u = [ con.cx_u(:,:,k) ; con.cu_u(:,:,k)];
    
    lamx = mult.lamx(:,k);
    lamu = mult.lamu(:,k);
    mudx = mult.mudx(:,k);
    mudu = mult.mudu(:,k);
    
    lam = [ lamx ; lamu];
    I_mu = diag([ mudx ; mudu ] );
    
    % Generate Intermediate Terms
    Qin = lqr.Qn;
    Rin = lqr.Rn;
    
    dQk = dQn(Qin,1);
    dRk = dRn(Rin,1);
    dqk = dqn(x,xs,Qin,1);
    drk = drn(u,us,Rin,1);
    
    Qx  = dqk + A'*v + c_x'*(lam + I_mu*c);
    Qu  = drk + B'*v + c_u'*(lam + I_mu*c);
    Qxx = dQk + A'*V*A + c_x'*I_mu*c_x;
    
    switch mode
        case 'fast'
            % Generate Feedback Update
            l(:,k)   = -Qu;
        case 'slow'
            Quu = dRk + B'*V*B + c_u'*I_mu*c_u + reg;
            while rcond(Quu) < 1e-5
                reg = rho.*eye(n_u);
                rho = 10.*rho;
                
                Quu = dRk + B'*V*B + c_u'*I_mu*c_u + reg;
            end
            Qux = B'*V*A + c_u'*I_mu*c_x;
            
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