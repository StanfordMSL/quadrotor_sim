function J = lagr_calc(x,u,con,lambda,mu_diag)

% Initialize Variables
N     = size(x,2);
J_obj_arr = zeros(1,N);
J_con_arr = zeros(1,N);

% Compute stagewise cost
for k = 1:N-1
    x_k = x(:,k);
    u_k = u(:,k);
    
    c = con(:,k);
    
    ld = lambda(:,k);
    md = mu_diag(:,k);
    
    J_obj_arr(1,k) = obj_cost_k(x_k,u_k,x_k,u_k);
    J_con_arr(1,k) = con_cost(c,ld,md);
end

% Terminal Case
x_k = x(:,N);
u_k = x(:,N);

c = con(:,N);

ld = lambda(:,N);
md = mu_diag(:,N);

J_obj_arr(1,N) = obj_cost_N(x_k,u_k,x_k,u_k);
J_con_arr(1,N) = con_cost(c,ld,md);

% Totals
J.obj = sum(J_obj_arr);
J.con = sum(J_con_arr);
J.tot = J.obj + J.con;

end