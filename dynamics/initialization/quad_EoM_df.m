function t_comp = quad_EoM_df(model)

tic

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = sym('x',[13 1],'real');

% Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = sym('u',[4 1],'real');
 
% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v = x(4:6,1);
q = x(7:10,1);
w = x(11:13,1);

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];                % Body Rate vee map

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

quad_func = 'dynamics/flight/quadcopter_df';
    
% Unpack
db = model.est;     
g     = db.g;
dt    = db.dt;
I     = db.I;
m     = db.m;

% Forces
F_g   = m.*[0 ; 0 ; -g];                                                % Gravity
F_t   = quatrot2([0 ; 0 ; u(1)],q);                                        % Thrust

% Torques
tau_mot  =  u(2:4,1);                                                 % Motor
tau_rot  = -cross(w,I*w);

% Dynamics Equations
p_dot = v;
v_dot = (1/m) .* ( F_g + F_t);
q_dot = 0.5*W*q;
w_dot = I\(tau_mot+tau_rot);
x_dot = [ p_dot ; v_dot ; q_dot ; w_dot];

% Discrete Update Equations
x_upd = x + dt.*x_dot;
x_upd(7:10,1) = x_upd(7:10,1)./norm(x_upd(7:10,1));    % ensure quaternion is still unit mag.

% Output Quadcopter Dynamics Function
x_upd = simplify(x_upd);

matlabFunction(x_upd,'File',quad_func,'vars',{x,u});
    
t_comp = toc;
end