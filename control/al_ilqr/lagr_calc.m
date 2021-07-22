function J = lagr_calc(X,U,xs,us,T,con,lambda,mu_diag)

% Initialize Variables
N     = size(X,2);
J_objs = zeros(1,N);
J_cons = zeros(1,N);

% Initialize the Mex Sensitive Variables
J.objs = zeros(1,N);
J.cons = zeros(1,N);
J.tots = zeros(1,N);
J.obj = 0;
J.con = 0;
J.tot = 0;

% Compute stagewise cost
for k = 1:N-1
    xk = X(:,k);
    uk = U(:,k);
    
    c = con(:,k);
    
    ld = lambda(:,k);
    md = mu_diag(:,k);
    
    if k < T
        J_objs(1,k) = dCn(xk,uk,xk,uk,xs,us,1);
    else
        J_objs(1,k) = dCT(xk,uk,xk,uk,xs,us,1);
    end
    
    J_cons(1,k) = con_cost(c,ld,md);
end

% Terminal
xk = X(:,N);

c = con(:,N);

ld = lambda(:,N);
md = mu_diag(:,N);

J_objs(1,N) = dCN(xk,xk,xs,1);
J_cons(1,N) = con_cost(c,ld,md);

% Totals
J.objs = J_objs;
J.cons = J_cons;
J.tots = J.objs + J.cons;
J.obj = sum(J_objs);
J.con = sum(J_cons);
J.tot = J.obj + J.con;

% disp(['[lagr_calc]: Obj. Cost: ',num2str(J.obj),' Con. Cost: ',num2str(J.con)]);

end