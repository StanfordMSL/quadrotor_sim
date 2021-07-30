function lagr_init(cost_mode,input_mode)

switch input_mode
    case 'wrench'
        n_x = 13;
        n_u = 4;
    case 'body_rate'
        n_x = 10;
        n_u = 4;
end

N_cx = 16;
N_cu = 8;

% Initialize Variables
x = sym('x',[n_x 1],'real');
u = sym('u',[n_u 1],'real');

x_bar = sym('x_bar',[n_x 1],'real');
u_bar = sym('u_bar',[n_u 1],'real');

x_star = sym('x_star',[n_x 1],'real');
u_star = sym('u_star',[n_u 1],'real');

lam_x = sym('lam_x',[N_cx 1],'real');
mud_x = sym('mud_x',[N_cx 1],'real');

lam_u = sym('lam_u',[N_cu 1],'real');
mud_u = sym('mud_u',[N_cu 1],'real');

conx = sym('conx',[N_cx 1],'real');
conu = sym('conu',[N_cu 1],'real');

trig = sym('trig','real');

% Standard Weights
Qp  = ones(3,1);
Qv  = ones(3,1);
Qq  = zeros(4,1);

sQ = diag([Qp ; Qv ; Qq]);
% sR = eye(n_u);

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

dCn = [del_x'*dQn*del_x + dqn'*del_x ;...
       del_u'*dRn*del_u + drn'*del_u ;...
             del_u'*dHn*del_x        ;...
                  dpn];
              
dCT = [del_x'*dQT*del_x + dqT'*del_x ;...
       del_u'*dRT*del_u + drT'*del_u ;...
             del_u'*dHT*del_x        ;...
                  dpT]; 

dCN = [del_x'*dQN*del_x + dqN'*del_x ;...
                   0                 ;...
                   0                 ;...
                  dpN]; 

% Augment with Trigger Variable
dCn_x  = trig.*dCn(1,1);
dCn_u  = trig.*dCn(2,1);
dCn_xu = trig.*dCn(3,1);
dCn_p  = trig.*dCn(4,1);

dCT_x  = trig.*dCT(1,1);
dCT_u  = trig.*dCT(2,1);
dCT_xu = trig.*dCT(3,1);
dCT_p  = trig.*dCT(4,1);

dCN_x  = trig.*dCN(1,1);
dCN_u  = trig.*dCN(2,1);
dCN_xu = trig.*dCN(3,1);
dCN_p  = trig.*dCN(4,1);

% Compute Constraint Costs
Imu_x = diag(mud_x);
Imu_u = diag(mud_u);
conx_cost = (lam_x + 0.5*Imu_x*conx)'*conx;
conu_cost = (lam_u + 0.5*Imu_u*conu)'*conu;
         
% Export Functions
add = 'control/al_ilqr/lagrangian/';

matlabFunction(dQN,'File',[add,'dQN'],'vars',trig);
matlabFunction(dqN,'File',[add,'dqN'],'vars',{x_bar,x_star,trig});

matlabFunction(dQT,'File',[add,'dQT'],'vars',trig);
matlabFunction(dRT,'File',[add,'dRT'],'vars',trig);
matlabFunction(dqT,'File',[add,'dqT'],'vars',{x_bar,x_star,trig});
matlabFunction(drT,'File',[add,'drT'],'vars',{u_bar,u_star,trig});

matlabFunction(dQn,'File',[add,'dQn'],'vars',trig);
matlabFunction(dRn,'File',[add,'dRn'],'vars',trig);
matlabFunction(dqn,'File',[add,'dqn'],'vars',{x_bar,x_star,trig});
matlabFunction(drn,'File',[add,'drn'],'vars',{u_bar,u_star,trig});

matlabFunction(dCn_x,'File',[add,'dCn_x'],'vars',{x,x_bar,x_star,trig});
matlabFunction(dCn_u,'File',[add,'dCn_u'],'vars',{u,u_bar,u_star,trig});
matlabFunction(dCn_xu,'File',[add,'dCn_xu'],'vars',{x,u,x_bar,u_bar,x_star,u_star,trig});
matlabFunction(dCn_p,'File',[add,'dCn_p'],'vars',{x_bar,u_bar,x_star,u_star,trig});

matlabFunction(dCT_x,'File',[add,'dCT_x'],'vars',{x,x_bar,x_star,trig});
matlabFunction(dCT_u,'File',[add,'dCT_u'],'vars',{u,u_bar,u_star,trig});
matlabFunction(dCT_xu,'File',[add,'dCT_xu'],'vars',{x,u,x_bar,u_bar,x_star,u_star,trig});
matlabFunction(dCT_p,'File',[add,'dCT_p'],'vars',{x_bar,u_bar,x_star,u_star,trig});

matlabFunction(dCN_x,'File',[add,'dCN_x'],'vars',{x,x_bar,x_star,trig});
matlabFunction(dCN_u,'File',[add,'dCN_u'],'vars',trig);
matlabFunction(dCN_xu,'File',[add,'dCN_xu'],'vars',trig);
matlabFunction(dCN_p,'File',[add,'dCN_p'],'vars',{x_bar,x_star,trig});

matlabFunction(conx_cost,'File',[add,'conx_cost'],'vars',{conx,lam_x,mud_x});
matlabFunction(conu_cost,'File',[add,'conu_cost'],'vars',{conu,lam_u,mud_u});

