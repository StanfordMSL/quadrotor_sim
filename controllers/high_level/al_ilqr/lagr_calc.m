function J = lagr_calc(X,U,xs,us,con,lambda,mu_diag)

% Initialize Variables
N     = size(X,2);
J_objs = zeros(1,N);
J_cons = zeros(1,N);

% Generate the nominals and ideals (Xstar, Ustar)
Xs = [X(:,2:end) xs];
Us = repmat(us,1,N-1);

% Compute stagewise cost
for k = 1:N-1
    xk = X(:,k);
    uk = U(:,k);    
    xs = Xs(:,k);
    us = Us(:,k);
    
    c = con(:,k);
    
    ld = lambda(:,k);
    md = mu_diag(:,k);
    
    J_objs(1,k) = obj_cost_k(xk,uk,xk,uk,xs,us);
    J_cons(1,k) = con_cost(c,ld,md);
end

% Terminal Case
xk = X(:,N);
uk = X(:,N);
xs = Xs(:,k);
us = Us(:,k);

c = con(:,N);

ld = lambda(:,N);
md = mu_diag(:,N);

J_objs(1,N) = obj_cost_N(xk,uk,xk,uk,xs,us);
J_cons(1,N) = con_cost(c,ld,md);

% Totals
J.objs = J_objs;
J.cons = J_cons;
J.tots = J.objs + J.cons;
J.obj = sum(J_objs);
J.con = sum(J_cons);
J.tot = J.obj + J.con;

end