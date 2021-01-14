function u_motor = df_con(f_out,model)

% Unpack Items
g    = [ 0 ; 0 ; model.g];
m    = model.m_est;
I    = model.I_est;
kh   = model.kh_est;
A    = model.A_est;
B    = model.B_est;
tau_g = 0.000.*ones(3,1);

D  = model.D_est;
kDx = D(1,1);
kDy = D(2,2);
kDz = D(3,3);

sigma    = f_out(:,1);
sigma_d1 = f_out(:,2);
sigma_d2 = f_out(:,3);
sigma_d3 = f_out(:,4);
sigma_d4 = f_out(:,5);

v_c = sigma_d1(1:3,1);
a_c = sigma_d2(1:3,1);
j_c = sigma_d3(1:3,1);
s_c = sigma_d4(1:3,1);

% Compute alpha and beta
alpha = a_c + g + kDx.*v_c;
beta  = a_c + g + kDy.*v_c;

% Compute x_c and y_c
x_c = [ cos(sigma(4,1)) ; sin(sigma(4,1)) ; 0];
y_c = [-sin(sigma(4,1)) ; cos(sigma(4,1)) ; 0];

% Compute x_b, y_b and z_b
x_b = cross(y_c,alpha)./norm(cross(y_c,alpha));
y_b = cross(beta,x_b)./norm(cross(beta,x_b));
z_b = cross(x_b,y_b);

R = [x_b y_b z_b];

% Compute c
c = z_b' * (a_c + g + kDz.*v_c);
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
D3 = sigma_d1(4,1) * x_c'*x_b;

% Compute w_x,w_y,w_z
w_x = (-(B1*C1*D3)+(B1*C3*D2)-(B3*C1*D2)+B3*C2*D1) / (A2*(B1*C3-B3*C1));
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
E3 =  sigma_d2(4,1)*x_c'*x_b...
     + 2*sigma_d1(4,1)*w_z*x_c'*y_b...
     - 2*sigma_d1(4,1)*w_y*x_c'*z_b...
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

% Compute motor inputs
T_motor = model.m2w_est_inv*wrench;
u_motor = sqrt(T_motor./model.kw_est(1,1));

end

