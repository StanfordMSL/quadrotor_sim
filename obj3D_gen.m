function [Jk,JN] = obj3D_gen()

nx = 13;
nu = 4;

% Initialize Variables
x = sym('x',[nx 1],'real');
u = sym('u',[nu 1],'real');

x_s = sym('x_s',[nx 1],'real');
u_s = sym('u_s',[nu 1],'real');

% Weight Inputs
Q = sym('Q',[nx 1],'real');
R = sym('R',[nu 1],'real');

% Stagewise Cost
Jk = 0.5.*((x-x_s)'*diag(Q)*(x-x_s) + (u-u_s)'*diag(R)*(u-u_s));

% Terminal Cost
JN = 0.5.*(x-x_s)'*diag(Q)*(x-x_s);

matlabFunction(Jk,'File','Jk_func','vars',{x,u,x_s,u_s,Q,R});
matlabFunction(JN,'File','JN_func','vars',{x,x_s,Q});


end
