function lagr_init(cost_mode,input_mode)

tic

switch input_mode
    case 'body_rate'
        n_x = 17;
        n_u = 4;
end

N_cx = 22;
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

% Weight Inputs
Qin = sym('Qin_d',[17 1],'real');
Rin  = sym('Rin_d',[4 1],'real');

disp('[lagr_init]: Assuming we are using l2-norm style costs');

switch cost_mode
    case 'terminal'        
        Qn = diag(Qin);
        Rn = diag(Rin);

        QN = diag(Qin);
end

% Full Form of Quadratic
dQn = Qn;
dRn = Rn;
dHn = zeros(n_u,n_x);
dqn = Qn*(x_bar - x_star);
drn = Rn*(u_bar - u_star);
dpn = 0.5.*(x_bar - x_star)'*Qn*(x_bar - x_star) +  0.5.*(u_bar - u_star)'*Rn*(u_bar - u_star);
     
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

dCN = [del_x'*dQN*del_x + dqN'*del_x ;...
                   0                 ;...
                   0                 ;...
                  dpN]; 

% Augment with Trigger Variable
dCn_x  = trig.*dCn(1,1);
dCn_u  = trig.*dCn(2,1);
dCn_xu = trig.*dCn(3,1);
dCn_p  = trig.*dCn(4,1);

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

matlabFunction(dQN,'File',[add,'dQN'],'vars',{Qin,trig});
matlabFunction(dqN,'File',[add,'dqN'],'vars',{x_bar,x_star,Qin,trig});

matlabFunction(dQn,'File',[add,'dQn'],'vars',{Qin,trig});
matlabFunction(dRn,'File',[add,'dRn'],'vars',{Rin,trig});
matlabFunction(dqn,'File',[add,'dqn'],'vars',{x_bar,x_star,Qin,trig});
matlabFunction(drn,'File',[add,'drn'],'vars',{u_bar,u_star,Rin,trig});

matlabFunction(dCn_x,'File',[add,'dCn_x'],'vars',{x,x_bar,x_star,Qin,trig});
matlabFunction(dCn_u,'File',[add,'dCn_u'],'vars',{u,u_bar,u_star,Rin,trig});
matlabFunction(dCn_xu,'File',[add,'dCn_xu'],'vars',{x,u,x_bar,u_bar,x_star,u_star,trig});
matlabFunction(dCn_p,'File',[add,'dCn_p'],'vars',{x_bar,u_bar,x_star,u_star,Qin,Rin,trig});

matlabFunction(dCN_x,'File',[add,'dCN_x'],'vars',{x,x_bar,x_star,Qin,trig});
matlabFunction(dCN_u,'File',[add,'dCN_u'],'vars',trig);
matlabFunction(dCN_xu,'File',[add,'dCN_xu'],'vars',trig);
matlabFunction(dCN_p,'File',[add,'dCN_p'],'vars',{x_bar,x_star,Qin,trig});

matlabFunction(conx_cost,'File',[add,'conx_cost'],'vars',{conx,lam_x,mud_x});
matlabFunction(conu_cost,'File',[add,'conu_cost'],'vars',{conu,lam_u,mud_u});

t_comp = toc;
disp(['[lagr_init]: Lagrangian Terms Generated in ' num2str(t_comp) 's']);


end

