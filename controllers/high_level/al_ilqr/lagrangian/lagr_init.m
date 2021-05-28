function lagr_init(cost_mode,input_mode,obj)

switch input_mode
    case 'wrench'
        n_x = 13;
        n_u = 4;
    case 'body_rate'
        n_x = 10;
        n_u = 4;
end
n_c = 24;

% Initialize Variables
syms x [n_x 1] real
syms u [n_u 1] real

syms x_bar [n_x 1] real
syms u_bar [n_u 1] real

syms lambda [n_c 1] real
syms mu_diag [n_c 1] real

syms con [n_c 1] real
syms con_x [n_c n_x] real
syms con_u [n_c n_u] real

syms rho real

del_x = x-x_bar;
del_u = u-u_bar;

x_star = obj.x(1:n_x,end);

I_mu = diag(mu_diag);

switch cost_mode
    case 'con_only'
        Q_k = zeros(n_x,n_x);
        R_k = zeros(n_u,n_u);
        q_k = zeros(n_x,1);
        r_k = zeros(n_u,1);
        H_k = zeros(n_u,n_x);
        p_k = 0;
        
        Q_N = zeros(n_x,n_x);
        q_N = zeros(n_x,1);
        p_N = 0;
    case 'terminal'
        Q_k = zeros(n_x,n_x);
        R_k = zeros(n_u,n_u);
        q_k = zeros(n_x,1);
        r_k = zeros(n_u,1);
        H_k = zeros(n_u,n_x);
        p_k = 0;
        
        Q_N = eye(n_x);
        q_N = ((x_bar - x_star)'*Q_N)';
        p_N = 0.5*(x_bar - x_star)'*Q_N*(x_bar - x_star);
end

% Cost Calculation
add = 'controllers/high_level/al_ilqr/lagrangian/';

obj_cost_k = del_x'*Q_k*del_x + del_u'*R_k*del_u + del_u'*H_k*x + q_k'*del_x + r_k'*del_u + p_k;
obj_cost_N = del_x'*Q_N*del_x + q_N'*del_x + p_N;

con_cost = (lambda + 0.5*I_mu*con)'*con;

matlabFunction(obj_cost_k,'File',[add,'obj_cost_k'],'vars',{x,u,x_bar,u_bar});
matlabFunction(obj_cost_N,'File',[add,'obj_cost_N'],'vars',{x,u,x_bar,u_bar});
matlabFunction(con_cost,'File',[add,'con_cost'],'vars',{con,lambda,mu_diag});

matlabFunction(Q_N,'File',[add,'Q_N_calc'],'vars',x_bar(1));
matlabFunction(q_N,'File',[add,'q_N_calc'],'vars',{x_bar});
matlabFunction(Q_k,'File',[add,'Q_k_calc'],'vars',x_bar(1));
matlabFunction(R_k,'File',[add,'R_k_calc'],'vars',u_bar(1));
matlabFunction(q_k,'File',[add,'q_k_calc'],'vars',x_bar(1));
matlabFunction(r_k,'File',[add,'r_k_calc'],'vars',u_bar(1));
