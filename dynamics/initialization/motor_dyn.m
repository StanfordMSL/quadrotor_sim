function t_comp = motor_dyn(model)

tic
% Note: the motor dynamics isn't accurate in reverse but this is
% still necessary as the functions NEED to be continous. The constraint
% will take care of the rest.

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = sym('x',[10 1],'real');

% Motor Rates (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wm = sym('wm',[4 1],'real');

% Body Rate Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = sym('u',[4 1],'real');
u_p = sym('u_p',[4 1],'real');

% Wrench %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wrench = sym('wrench',[4 1],'real');

% Thrust Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f  = sym('f','real');
fn = sym('fn','real');

% Unpack Some Stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt      = model.est.dt;
A       = model.est.A;
B       = model.est.B;
I       = model.est.I;
w2m     = model.est.w2m;
val_min = model.motor.thrust_min;
val_max = model.motor.thrust_max;
kw      = model.est.kw;
Qm      = [wm.^0 wm.^1 wm.^2];                  % quadratic structure of motor rot speed

% Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

w   = u(2:4,1);
v   = x(4:6,1);                             % Velocity States
q   = x(7:10,1);                            % Quaternion States
q_c = [ q(1) ; -q(2) ; -q(3) ; -q(4) ];     % Conjugate quaternion

Jxx = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jyy = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jzz = (model.motor.m/2)*(model.motor.r1^2+model.motor.r0^2);
J = diag([Jxx Jyy Jzz]);                    % Motor Inertia Tensor

vb = quatrot2(v,q_c);                       % velocity in body frame

Qvb = sign(vb).*[vb.^0 abs(vb) vb.^2];      % quadratic structure of craft linear velocity (body frame)
Qw = sign(w).*[w.^0     abs(w)   w.^2];     % quadratic structure of craft angular velocity
Wm = [0       0      0      0;
      0       0      0      0;
    -wm(1) -wm(2) wm(3) wm(4)];             % Motor rotational speed map

PwWm = [...
    cross(w,Wm(:,1))...
    cross(w,Wm(:,2))...
    cross(w,Wm(:,3))...
    cross(w,Wm(:,4))];                      % Cross products of motor and craft angular velocities

% Torque Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau_rot  = -cross(w,I*w);                                               % Rotating Frame
tau_linD = -[ A(1,:)*Qvb(1,:)' ; A(2,:)*Qvb(2,:)'; A(3,:)*Qvb(3,:)'];   % Linear Velocity Drag
tau_accD = -[ B(1,:)*Qw(1,:)'  ; B(2,:)*Qw(2,:)' ; B(3,:)*Qw(3,:)'];    % Angular Velocity Drag
tau_g    = -J*(sum(PwWm,2));                                            % Motor Precession

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% From Motor Angular Velocity to Individual Motor Thrusts
F_m = sign(wm).*(Qm*kw);

% From Wrench to Motor Angular Velocity
mt = w2m*wrench;
mt_sgn = sign(mt);
mt_abs = abs(mt);
u_m = mt_sgn.*sqrt(mt_abs./kw(3));          

% From body rate input to wrench
f_mot   = 4*u(1,1)*(val_max-val_min) + val_min; 
w_dot   = (u(2:4,1) - u_p(2:4,1))./dt;
tau_mot = (I*w_dot) - tau_rot;
% tau_mot = (I*w_dot) - tau_rot - tau_g - tau_linD - tau_accD;
disp('[motor_dyn]: Using simple wrench dynamics. change me.');
br2wr   = [f_mot ; tau_mot];

% Output Thrust (f) to Normalized Thrust (fn)
fn_out = (1/4) * ((f-val_min)/(val_max-val_min));
f_out  = 4*fn*(val_max-val_min) + val_min;

% Output Functions
matlabFunction(F_m,'File','dynamics/motor_mapping/m2f','vars',{wm});
matlabFunction(u_m,'File','dynamics/motor_mapping/w2m','vars',{wrench});
matlabFunction(br2wr,'File','dynamics/motor_mapping/br2wr','vars',{x,u,u_p,wm});
matlabFunction(fn_out,'File','dynamics/motor_mapping/f2fn','vars',f);
matlabFunction(f_out,'File','dynamics/motor_mapping/fn2f','vars',fn);

t_comp = toc;
end