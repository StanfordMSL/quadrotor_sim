function lagr_init(cost_mode,input_mode)

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

syms x_star [n_x 1] real
syms u_star [n_u 1] real

syms lambda [n_c 1] real
syms mu_diag [n_c 1] real

syms con [n_c 1] real
syms con_x [n_c n_x] real
syms con_u [n_c n_u] real

syms rho real

syms a real
% Standard Weights
sQ = eye(n_x);
sR = eye(n_u);

% Empty Weights
eQ = zeros(n_x,n_x);
eR = zeros(n_u,n_u);

disp('[lagr_init]: Assuming we are using l2-norm style costs');

switch cost_mode
    case 'con_only'
        hQ_N = eQ;
        hQ_k = eQ;
        
        hR_k = eR;
    case 'terminal'
        hQ_N = sQ;
        hQ_k = eQ;
        
        hR_k = eR;        
    case 'min_time'
        hQ_N = sQ;
        hQ_k = sQ;
        
        hR_k = eR;
    case 'min_energy'
        hQ_N = sQ;
        hQ_k = eQ;
        
        hR_k = sR;
end

% Full Form of Quadratic
Q_N = hQ_N;
q_N = hQ_N*(x_bar - x_star);
p_N = 0.5.*(x_bar - x_star)'*hQ_N*(x_bar - x_star);

Q_k = hQ_k;
q_k = hQ_k*(x_bar - x_star);

R_k = hR_k;
r_k = hR_k*(u_bar - u_star);

H_k = zeros(n_u,n_x);

p_k = 0.5.*(x_bar - x_star)'*hQ_k*(x_bar - x_star) +  0.5.*(u_bar - u_star)'*hR_k*(u_bar - u_star);
        
% Compute Objective Costs
del_x = x-x_bar;
del_u = u-u_bar;

obj_cost_k = del_x'*Q_k*del_x + del_u'*R_k*del_u + del_u'*H_k*x + q_k'*del_x + r_k'*del_u + p_k;
obj_cost_N = del_x'*Q_N*del_x + q_N'*del_x + p_N;

% Compute Constraint Costs
I_mu = diag(mu_diag);
con_cost = (lambda + 0.5*I_mu*con)'*con;

% Export Functions
add = 'control/al_ilqr/lagrangian/';

matlabFunction(obj_cost_k,'File',[add,'obj_cost_k'],'vars',{x,u,x_bar,u_bar,x_star,u_star});
matlabFunction(obj_cost_N,'File',[add,'obj_cost_N'],'vars',{x,u,x_bar,u_bar,x_star,u_star});
matlabFunction(con_cost,'File',[add,'con_cost'],'vars',{con,lambda,mu_diag});

matlabFunction(Q_N,'File',[add,'Q_N_calc'],'vars',a);
matlabFunction(q_N,'File',[add,'q_N_calc'],'vars',{x_bar,x_star});
matlabFunction(Q_k,'File',[add,'Q_k_calc'],'vars',a);
matlabFunction(R_k,'File',[add,'R_k_calc'],'vars',a);
matlabFunction(q_k,'File',[add,'q_k_calc'],'vars',{x_bar,x_star});
matlabFunction(r_k,'File',[add,'r_k_calc'],'vars',{u_bar,x_star,u_star});
