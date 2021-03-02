function u_motor = att_pos_simple(x_now,u_df,f_out,model)     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller Constants
Kp = 0;
Kv = 0;
Kr = 1.0.*eye(3);
Kw = 1.0.*eye(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Unpack Items
g    = [ 0 ; 0 ; model.g];
m    = model.m_est;
I    = model.I_est;

%%%%%%%%%%%%%%%
kh   = model.kh_est;
A    = model.A_est;
B    = model.B_est;
tau_g = 0.000.*ones(3,1);

D   = model.D_est;
kDx = D(1,1);
kDy = D(2,2);
kDz = D(3,3);

psi      = f_out(4,1);
psi_dot  = f_out(4,2);
psi_ddot = f_out(4,3);

v_c = f_out(1:3,2);
a_c = f_out(1:3,3);
j_c = f_out(1:3,4);
s_c = f_out(1:3,5);

% Compute alpha and beta
alpha = a_c + g + kDx.*v_c;
beta  = a_c + g + kDy.*v_c;

% Compute x_c and y_c
x_c = [ cos(psi) ; sin(psi) ; 0];
y_c = [-sin(psi) ; cos(psi) ; 0];

% Compute x_b, y_b and z_b
x_b = cross(y_c,alpha)./norm(cross(y_c,alpha));
y_b = cross(beta,x_b)./norm(cross(beta,x_b));
z_b = cross(x_b,y_b);

R = [x_b y_b z_b];

% Compute c
c     = z_b' * (a_c + g + kDz.*v_c);
c_cmd = c - kh*(v_c'*(x_b+y_b))^2;

% Find Main Intermediate Terms
B1 = c - (kDz-kDx)*(z_b'*v_c);
C1 = -(kDx-kDy)*(y_b'*v_c);
D1 = x_b'*j_c + kDx*x_b'*a_c;
A2 = c + (kDy - kDz)*(z_b'*v_c);
C2 = (kDx-kDy)*(x_b'*v_c);
D2 = -y_b'*j_c - kDy*y_b'*a_c;
B3 = -y_c'*z_b;
C3 = norm(cross(y_c,z_b));
D3 = psi_dot * x_c'*x_b;

% Compute w_x,w_y,w_z
w_x = (-(B1*C1*D3)+(B1*C3*D2)-(B3*C1*D2)+(B3*C2*D1)) / (A2*(B1*C3-B3*C1));
w_y = (-(C1*D3)+(C3*D1)) / ((B1*C3)-(B3*C1));
w_z = ( (B1*D3)-(B3*D1)) / ((B1*C3)-(B3*C1));

w = [ w_x ; w_y ; w_z];

% Compute c_dot
c_dot = z_b'*j_c + w_x*(kDy-kDz)*(y_b'*v_c) + w_y*(kDz-kDx)*(x_b'*v_c) + kDz*z_b'*a_c;

% Compute eta
w_acc = [ 0.0 -w_z  w_y ;...
          w_z  0.0 -w_x ;...
         -w_y  w_x  0.0];

eta = R*(w_acc*w_acc*D + D*w_acc*w_acc + 2*w_acc*D*w_acc')*R'*v_c +...
      2*R*(w_acc*D + D*w_acc')*R'*a_c +...
      R*D*R'*j_c;

% Compute Remainder of Intermediate Terms
E1 =  x_b'*s_c - 2*c_dot*w_y - c*w_x*w_z + x_b'*eta;
E2 = -y_b'*s_c - 2*c_dot*w_x + c*w_y*w_z - y_b'*eta;
E3 =  psi_ddot*x_c'*x_b...
     + 2*psi_dot*w_z*x_c'*y_b...
     - 2*psi_dot*w_y*x_c'*z_b...
     - w_x*w_y*y_c'*y_b...
     - w_x*w_z*y_c'*z_b;

% Find w_dot_x, w_dot_y, w_dot_z
w_dot_x = ( -(B1*C2*E3)+(B1*C3*E2)-(B3*C1*E2)+(B3*C2*E1) ) / (A2*((B1*C3)-(B3*C1)));
w_dot_y = ( -(C1*E3)+(C3*E1) )/ ( (B1*C3)-(B3*C1) );
w_dot_z = (  (B1*E3)-(B3*E1) )/ ( (B1*C3)-(B3*C1) );
       
w_dot = [w_dot_x ; w_dot_y ; w_dot_z];

% Compute wrench
f_wrench   = m*c_cmd;
tau_wrench = I*w_dot + A*R'*v_c + B*w + cross(w,I*w) + tau_g;
wrench     = [f_wrench ; tau_wrench];

% Directly convert wrench to motor inputs
T_motor = model.m2w_est_inv*u_w(:,k);
u_m(:,k) = sqrt(T_motor./model.kw_est(1,1));

end
        