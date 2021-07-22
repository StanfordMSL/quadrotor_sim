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
x = sym('x',[n_x 1],'real');
u = sym('u',[n_u 1],'real');

x_bar = sym('x_bar',[n_x 1],'real');
u_bar = sym('u_bar',[n_u 1],'real');

x_star = sym('x_star',[n_x 1],'real');
u_star = sym('u_star',[n_u 1],'real');

lambda = sym('lambda',[n_c 1],'real');
mu_diag = sym('mu_diag',[n_c 1],'real');

con = sym('con',[n_c 1],'real');
trig = sym('trig','real');

% Standard Weights
sQ = eye(n_x);
sR = eye(n_u);

% Empty Weights
eQ = zeros(n_x,n_x);
eR = zeros(n_u,n_u);

disp('[lagr_init]: Assuming we are using l2-norm style costs');

switch cost_mode
    case 'terminal'        
        Qn = eQ;
        QT = sQ;
        QN = sQ;

        Rk = eR;
        RT = eR;
        
%     case 'con_only'
%         hQ_N = eQ;
%         hQ_k = eQ;
%         
%         hR_k = eR;
%     case 'min_time'
%         hQ_N = sQ;
%         hQ_k = sQ;
%         
%         hR_k = eR;
%     case 'min_energy'
%         hQ_N = sQ;
%         hQ_k = eQ;
%         
%         hR_k = sR;
end

% Full Form of Quadratic
dQn = Qn;
dRn = Rk;
dHn = zeros(n_u,n_x);
dqn = Qn*(x_bar - x_star);
drn = Rk*(u_bar - u_star);
dpn = 0.5.*(x_bar - x_star)'*Qn*(x_bar - x_star) +  0.5.*(u_bar - u_star)'*Rk*(u_bar - u_star);

dQT = QT;
dRT = RT;
dHT = zeros(n_u,n_x);
dqT = QT*(x_bar - x_star);
drT = RT*(u_bar - u_star);
dpT = 0.5.*(x_bar - x_star)'*QT*(x_bar - x_star) +  0.5.*(u_bar - u_star)'*RT*(u_bar - u_star);
        
dQN = QN;
dqN = QN*(x_bar - x_star);
dpN = 0.5.*(x_bar - x_star)'*QN*(x_bar - x_star);

% Compute Objective Costs
del_x = x-x_bar;
del_u = u-u_bar;

dCn = del_x'*dQn*del_x + del_u'*dRn*del_u + del_u'*dHn*x + dqn'*del_x + drn'*del_u + dpn;
dCT = del_x'*dQT*del_x + del_u'*dRT*del_u + del_u'*dHT*x + dqT'*del_x + drT'*del_u + dpT;
dCN = del_x'*dQN*del_x + dqN'*del_x + dpN;

% Augment with Trigger Variable
dCn = trig.*dCn;
dCT = trig.*dCT;
dCN = trig.*dCN;

% Compute Constraint Costs
I_mu = diag(mu_diag);
con_cost = (lambda + 0.5*I_mu*con)'*con;

% Export Functions
add = 'control/al_ilqr/lagrangian/';

matlabFunction(dQN,'File',[add,'dQN_calc'],'vars',trig);
matlabFunction(dqN,'File',[add,'dqN_calc'],'vars',{x_bar,x_star,trig});

matlabFunction(dQT,'File',[add,'dQT_calc'],'vars',trig);
matlabFunction(dRT,'File',[add,'dRT_calc'],'vars',trig);
matlabFunction(dqT,'File',[add,'dqT_calc'],'vars',{x_bar,x_star,trig});
matlabFunction(drT,'File',[add,'drT_calc'],'vars',{u_bar,u_star,trig});

matlabFunction(dQn,'File',[add,'dQn_calc'],'vars',trig);
matlabFunction(dRn,'File',[add,'dRn_calc'],'vars',trig);
matlabFunction(dqn,'File',[add,'dqn_calc'],'vars',{x_bar,x_star,trig});
matlabFunction(drn,'File',[add,'drn_calc'],'vars',{u_bar,u_star,trig});

matlabFunction(dCn,'File',[add,'dCn'],'vars',{x,u,x_bar,u_bar,x_star,u_star,trig});
matlabFunction(dCT,'File',[add,'dCT'],'vars',{x,u,x_bar,u_bar,x_star,u_star,trig});
matlabFunction(dCN,'File',[add,'dCN'],'vars',{x,x_bar,x_star,trig});

matlabFunction(con_cost,'File',[add,'con_cost'],'vars',{con,lambda,mu_diag});

