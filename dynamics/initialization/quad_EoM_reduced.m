function [t_comp,x_upd] = quad_EoM_reduced(model)

tic

% Unpack Some Stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g     = model.est.g;
dt    = model.est.dt;
D     = model.est.D;
m     = model.est.m;

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = sym('x',[10 1],'real');

p = x(1:3,1);
v = x(4:6,1);
q = x(7:10,1);

% Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u    = sym('u',[4 1],'real');

u_c  = u(1);
u_br = u(2:4);
    
% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

W = [  0  -u_br(1) -u_br(2) -u_br(3) ;
     u_br(1)   0    u_br(3) -u_br(2) ;
     u_br(2) -u_br(3)    0   u_br(1) ;
     u_br(3)  u_br(2) -u_br(1)    0];                   % Body Rate vee map

Qvw = sign(v).*[v.^0 abs(v) v.^2];                      % quadratic structure of craft linear velocity (world frame)


% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forces
F_g   = m.*[0 ; 0 ; -g];                                                % Gravity
F_t   = quatrot2([0 ; 0 ; fn2f(u_c)],q);                                % Thrust
F_D   = -[ D(1,:)*Qvw(1,:)' ; D(2,:)*Qvw(2,:)' ; D(3,:)*Qvw(3,:)'];     % Drag

% Dynamics Equations
p_dot = v;
v_dot = (1/m) .* ( F_g + F_t + F_D);
q_dot = 0.5*W*q;

x_dot = [ p_dot ; v_dot ; q_dot ];

% Discrete Update Equations
x_upd = x + dt.*x_dot;

x_upd(7:10,1)  =  x_upd(7:10,1)./norm(x_upd(7:10,1));        % ensure quaternion is still unit mag.

% Output Quadcopter Dynamics Function
x_upd = simplify(x_upd);

t_comp = toc;

end