function [t_comp,z_upd] = err_dyn(model)
tic

% Unpack Some Stuff
dt    = model.est.dt;

% Integral Error Variables (tracking position and quaternion)
x = sym('x',[10 1],'real');

p = x(1:3);
q = x(7:10);
p_bar = sym('p_bar',[3 1],'real');
q_bar = sym('q_bar',[4 1],'real');

z = sym('z',[7 1],'real');

% Integral Error Dynamics
z_dot = [ p-p_bar ; q-q_bar ];
z_upd = z + dt.*z_dot;

% Output Error Dynamics
matlabFunction(z_upd,'File','dynamics/traj_error/error_upd','vars',{z,x,p_bar,q_bar});

t_comp = toc;

end