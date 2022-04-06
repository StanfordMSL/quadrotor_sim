function dyn_gen(model)

dt = model.dt;
g  = model.g;
m  = model.m;
l  = model.l;

x  = sym('x0',[2 1],'real');     % theta, theta_dot
th = sym('th',[2 1],'real');     % b, l

x_dot = [  x(2)  ; 
           (-m*g*th(2)*sin(x(1)) - th(1)*x(2))/(m*l^2)];
x_upd = x + dt.*x_dot;

A = jacobian(x_upd,x);
B = jacobian(x_upd,th);

matlabFunction(x_upd,'File','x_calc','vars',{x,th});
matlabFunction(A,'File','A_calc','vars',{x,th});
matlabFunction(B,'File','B_calc','vars',{x,th});

end