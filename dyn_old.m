function [A,B,x_opt] = dyn_old()

model = model_init('carlito','mismatch','precise');  
db = model.est;

p = sym('p',[3 1],'real');
v = sym('v',[3 1],'real');
q = sym('q',[4 1],'real');
w = sym('w',[3 1],'real');
W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];
F_ext = sym('F_ext',[3 1],'real');


% Unpack
g     = db.g;
dt    = db.dt;
D     = db.D;
m     = db.m;

x = sym('x',[10 1],'real');
u = sym('u',[4 1],'real');

% Forces
F_g =  m.*[0 ; 0 ; -g];
F_t =  quatrot2([0 ; 0 ; fn2f(u(1))],q);
v_D = sign(v).*[v.^0 abs(v) v.^2];
F_D = -[...
    D(1,:)*v_D(1,:)';...
    D(2,:)*v_D(2,:)';...
    D(3,:)*v_D(3,:)'];
F_D = simplify(F_D);

% Dynamic Equations
p_dot = v;
v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
q_dot = 0.5*W*q;

x_opt(1:3,1) = p + dt.*p_dot;
x_opt(4:6,1) = v + dt.*v_dot;

q_hat = q + dt.*q_dot;
x_opt(7:10,1)  = q_hat./norm(q_hat);

x_opt = subs(x_opt,p,x(1:3));
x_opt = subs(x_opt,v,x(4:6));
x_opt = subs(x_opt,q,x(7:10));
x_opt = subs(x_opt,w,u(2:4));

A = jacobian(x_opt,x);
B = jacobian(x_opt,u);




end
