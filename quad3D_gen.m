function [x_dot,x_upd] = quad3D_gen(model)

% States and Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = sym('x',[13 1],'real');                 % States
u = sym('u',[4 1],'real');                  % Inputs  

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q = x(4:7,1);                               % Quaternion
v = x(8:10,1);                              % Linear Velocity
w = x(11:13,1);                             % Angular Velocity

q_c = [ q(1) ; -q(2) ; -q(3) ; -q(4) ];     % Conjugate Quaternion

Imtr = model.motor.I;                       % Motor Inertia Tensor

vb = quatrot2(v,q_c);                       % velocity in body frame

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];                % Body Rate vee map

Qvw = sign(v).*[v.^0 abs(v) v.^2];          % quadratic structure of craft linear velocity (world frame)

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

db = model.est;

% Unpack
dt    = db.dt;
g     = db.g;
D     = db.D;
m     = db.m;
A     = db.A;
B     = db.B;
Iqd   = db.I;
wr2fm = db.wr2fm;
kw    = db.kw;

% Intermediate Terms
fm     = wr2fm*u;
mt_sgn = sign(fm);
mt_abs = abs(fm);
wm     = mt_sgn.*sqrt(mt_abs./kw(3));
Qvb    = sign(vb).*[vb.^0 abs(vb) vb.^2];      % quadratic structure of craft linear velocity (body frame)
Qw     = sign(w).*[w.^0     abs(w)   w.^2];     % quadratic structure of craft angular velocity
Wm     = [
    0       0      0      0;
    0       0      0      0;
    -wm(1) -wm(2) wm(3) wm(4)];               % Motor rotational speed map

PwWm   = [
    cross(w,Wm(:,1))...
    cross(w,Wm(:,2))...
    cross(w,Wm(:,3))...
    cross(w,Wm(:,4))];                      % Cross products of motor and craft angular velocities

% Forces
F_g   = m.*[0 ; 0 ; -g];                    % Gravity
F_t   = quatrot2([0 ; 0 ; u(1)],q);         % Thrust
F_D   = -[
    D(1,:)*Qvw(1,:)' ;
    D(2,:)*Qvw(2,:)' ;
    D(3,:)*Qvw(3,:)'];                      % Drag

% Torques
tau_wr   = u(2:4,1);                        % Wrench Input
tau_rot  = -cross(w,Iqd*w);                 % Rotating Frame
tau_linD = -[
    A(1,:)*Qvb(1,:)';
    A(2,:)*Qvb(2,:)';
    A(3,:)*Qvb(3,:)'];                      % Linear Velocity Drag
tau_accD = -[
    B(1,:)*Qw(1,:)';
    B(2,:)*Qw(2,:)';
    B(3,:)*Qw(3,:)'];                       % Angular Velocity Drag
tau_g    = -Imtr*(sum(PwWm,2));

% Dynamics Equations
p_dot = v;
q_dot = 0.5*W*q;
v_dot = (1/m) .* ( F_g + F_t + F_D);
w_dot = Iqd\(tau_wr + tau_rot + tau_g + tau_linD + tau_accD);

x_dot = [ p_dot ; q_dot ; v_dot ; w_dot];

x_upd = x + dt.*x_dot;
x_upd(4:7,1) = x_upd(4:7,1)./norm(x_upd(4:7,1));

matlabFunction(x_dot,'File','quad3Dc_func','vars',{x,u});
matlabFunction(x_upd,'File','quad3Dd_func','vars',{x,u});

end