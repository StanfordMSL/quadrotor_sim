function g = con2D_gen(X,p,eps)

opti = casadi.Opti();

g_pos = -norm(X(1:2)-p)+eps;
g_vel =  norm(X(4:5)) - 3;

g = [g_pos;g_vel];
% matlabFunction(g ,'File','g_func','vars',{x,p,eps});
