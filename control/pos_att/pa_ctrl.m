function u_wr = pa_ctrl(x_now,f_out,model)     

% Some useful constants
Kp = [ 1.00 0.00 0.00 ;
       0.00 1.00 0.00 ;
       0.00 0.00 1.80];
Kv = [ 1.00 0.00 0.00 ;
       0.00 1.00 0.00 ;
       0.00 0.00 1.80];
Kr = 0.1.*eye(3);
Kw = 0.1.*eye(3);


e1 = [ 1 ; 0 ; 0];
e2 = [ 0 ; 1 ; 0];
e3 = [ 0 ; 0 ; 1];
eta = 1e-9;

% Unpack some stuff
q = x_now(7:10,1);
q_c = [q(1) ; -q(2) ; -q(3) ; -q(4)];
p_now = x_now(1:3,1);
v_now = x_now(4:6,1);
R_now = quat2rotm(q');
w_now = x_now(11:13,1);

x_b = R_now*e1;
y_b = R_now*e2;
z_b = R_now*e3;
z_W = e3;

g    = [ 0 ; 0 ; model.g];
m    = model.m;

p_des = f_out(1:3,1);
v_des = f_out(1:3,2);
a_des = f_out(1:3,3);
j_des = f_out(1:3,4);

psi_des     = f_out(4,1);
psi_des_dot = f_out(4,2);

% Compute Desired Thrust
e_p = p_now - p_des;
e_v = v_now - v_des;

F_des = -Kp*e_p - Kv*e_v + m.*g + m.*a_des;
u1    = dot(F_des,z_b);

% Compute Desired Torque
z_b_des = F_des./(norm(F_des)+eta);

x_c_des = [ cos(psi_des) ; sin(psi_des) ; 0];

y_b_des = cross(z_b_des,x_c_des)./norm(cross(z_b_des,x_c_des));
x_b_des = cross(y_b_des,z_b_des);

R_des = [x_b_des y_b_des z_b_des];

R_err = 0.5.*(R_des'*R_now-R_now'*R_des);
e_R   = [R_err(3,2) ; R_err(1,3) ; R_err(2,1)];

h_w = (m/u1)*(j_des - (dot(z_b,j_des).*z_b));
w_des = [-dot(h_w,y_b) ; dot(h_w,x_b) ; psi_des_dot.*dot(z_W,z_b)]; 
    
e_w   = w_now-w_des;

u_tau = -Kr*e_R - Kw*e_w;

% Compile the Output
u_wr = [ u1 ; u_tau];

end
        