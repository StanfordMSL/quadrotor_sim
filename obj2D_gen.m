function J = obj2D_gen(N)

import casadi.*

% Dimensions
nx = 6;
nu = 6;

% Initialize Variables
x = MX.sym('x',nx,N);
u = MX.sym('u',nu,N-1);

x_s = MX.sym('x_s',nx,N);
u_s = MX.sym('u_s',nu,N-1);

% Weight Inputs
Q = MX.sym('Q',nx,N);
R = MX.sym('R',nu,N-1);

% Objective
J = 0;
for k = 1:N-1
    Jx = 0.5.*(x(:,k)-x_s(:,k))'*diag(Q(:,k))*(x(:,k)-x_s(:,k));
    Ju = 0.5.*(u(:,k)-u_s(:,k))'*diag(R(:,k))*(u(:,k)-u_s(:,k));
    J   = J+ (Jx + Ju);
end
Jx = 0.5.*(x(:,N)-x_s(:,N))'*diag(Q(:,N))*(x(:,N)-x_s(:,N));
J  = J+Jx;

% matlabFunction(J,'File','J_func','vars',{x,u,x_s,u_s,Q,R});

end
