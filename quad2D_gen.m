function [X,U,x_dot,x_upd] = quad2D_gen(model,N)

opti = casadi.Opti();

% States and Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = opti.variable(6,N+1);     % States
U = opti.variable(2,N);     % Inputs  

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

th  = x(3);
v   = [x(4) ; x(5)];   % Linear Velocity
thd = x(6);            % Angular Velocity

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

db = model.est;

% Unpack
dt    = db.dt;
g     = db.g;
m     = db.m;
Iqd   = db.I(2,2);

% Forces
F_g   = m.*[0 ; -g];                        % Gravity
F_t   = [-u(1)*sin(th) ; u(1)*cos(th)];     % Thrust

% Torques
tau_wr   = u(2);                            % Torque Input

% Dynamics Equations
p_dot   = v;
th_dot  = thd;
v_dot   = (1/m) .* ( F_g + F_t);
thd_dot = tau_wr/Iqd;

x_dot = [ p_dot ; th_dot ; v_dot ; thd_dot];

x_upd = x + dt.*x_dot;

% matlabFunction(x_dot,'File','quad2Dc_func','vars',{x,u});
% matlabFunction(x_upd,'File','quad2Dd_func','vars',{x,u});

end