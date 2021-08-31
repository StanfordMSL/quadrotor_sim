function t_comp = motor_dyn(model)

tic
% Note: the motor dynamics isn't accurate in reverse but this is
% still necessary as the functions NEED to be continous. The constraint
% will take care of the rest.

% Motor Inputs (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wm = sym('wm',[4 1],'real');

% Wrench %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wrench = sym('wrench',[4 1],'real');

% Thrust Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f  = sym('f','real');
fn = sym('fn','real');

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Unpack Some Stuff
w2m     = model.est.w2m;
val_min = model.motor.thrust_min;
val_max = model.motor.thrust_max;
kw      = model.est.kw;
Qm      = [wm.^0 wm.^1 wm.^2];                  % quadratic structure of motor rot speed


% From Motor Angular Velocity to Individual Motor Thrusts
F_m = sign(wm).*(Qm*kw);

% From Wrench to Motor Angular Velocity
mt = w2m*wrench;
mt_sgn = sign(mt);
mt_abs = abs(mt);
u_m = mt_sgn.*sqrt(mt_abs./kw(3));          

% Output Thrust (f) to Normalized Thrust (fn)
fn_out = (1/4) * ((f-val_min)/(val_max-val_min));
f_out  = 4*fn*(val_max-val_min) + val_min;

% Output Functions
matlabFunction(F_m,'File','dynamics/motor_mapping/m2f','vars',{wm});
matlabFunction(u_m,'File','dynamics/motor_mapping/w2m','vars',{wrench});
matlabFunction(fn_out,'File','dynamics/motor_mapping/f2fn','vars',f);
matlabFunction(f_out,'File','dynamics/motor_mapping/fn2f','vars',fn);

t_comp = toc;
end