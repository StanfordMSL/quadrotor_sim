function L = bp_exp(X,U,lqr,con,mult)

% Unpack some useful stuff
n_x = size(X,1);
n_u = size(U,1);
N   = lqr.N;

% Tuning Parameter
rho = 0.001;
reg = zeros(n_u,n_u);

% Initialize feedback policy variables
L = zeros(n_u,n_x,N-1);

% Initial
Qin = lqr.QN;
V = dQN(Qin,1);

for k = N-1:-1:1
    % Unpack stagewise stuff
    x = X(:,k);
    u = U(:,k);

    A = A_calc(x,u);
    B = B_calc(x,u);
    
    c_x = [ con.cx_x(:,:,k) ; con.cu_x(:,:,k)];
    c_u = [ con.cx_u(:,:,k) ; con.cu_u(:,:,k)];
    
    mudx = mult.mudx(:,k);
    mudu = mult.mudu(:,k);
    
    I_mu = diag([ mudx ; mudu ] );
    
    % Generate Intermediate Terms
    Qin = lqr.Qn;
    Rin = lqr.Rn;
    
    dQk = dQn(Qin,1);
    dRk = dRn(Rin,1);
    
    Qxx = dQk + A'*V*A + c_x'*I_mu*c_x;
 
    Quu = dRk + B'*V*B + c_u'*I_mu*c_u + reg;
    while rcond(Quu) < 1e-1
        reg = rho.*eye(n_u);
        rho = 10.*rho;
        
        Quu = dRk + B'*V*B + c_u'*I_mu*c_u + reg;
    end
    Qux = B'*V*A + c_u'*I_mu*c_x;
    
    L(:,:,k) = -Quu\Qux;
    
    V = Qxx + L(:,:,k)'*Quu*L(:,:,k) + L(:,:,k)'*Qux + Qux'*L(:,:,k);
end

end