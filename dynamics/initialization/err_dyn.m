function [t_comp,z_upd] = err_dyn(model)
tic

% Unpack Some Stuff
dt    = model.est.dt;

% Integral Error Variables (tracking position and quaternion)
x = sym('x',[9 1],'real');

p = x(1:3);
qv = x(8:10);
p_bar = sym('p_bar',[3 1],'real');
qv_bar = sym('qv_bar',[3 1],'real');

z = sym('z',[6 1],'real');

% Integral Error Dynamics
z_dot = [ p-p_bar ; qv-qv_bar ];
z_upd = z + dt.*z_dot;

% Output Error Dynamics
matlabFunction(z_upd,'File','dynamics/traj_error/error_upd','vars',{z,x,p_bar,qv_bar});

t_comp = toc;

end