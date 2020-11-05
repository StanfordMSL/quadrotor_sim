function u_cmd = df_con(f_out,model)

% Tuning Parameter
% kh = 0.00;
% A  = 0.00.*eye(3,3);
% B  = 0.00.*eye(3,3);
% tau_g = 0.000.*ones(3,1);

% kh = 0.009;
% A = -0.00001.*eye(3,3);
% B =  0.00001.*eye(3,3);
% tau_g = 0.000.*ones(3,1);

kh = 0.0001;
A  = 0.00001.*eye(3,3);
B  = 0.00001.*eye(3,3);
tau_g = 0.000.*ones(3,1);

% Unpack Items
g = [ 0 ; 0 ; model.g];
m = model.m_act;
I = model.I_act;
kd = model.kd_est;
D  = kd.*eye(3);

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
alpha = a_c + g + kd.*v_c;
beta  = a_c + g + kd.*v_c;

% Compute x_c and y_c
x_c = [ cos(sigma(4,1)) ; sin(sigma(4,1)) ; 0];
y_c = [-sin(sigma(4,1)) ; cos(sigma(4,1)) ; 0];

% Compute x_b, y_b and z_b
x_b = cross(y_c,alpha)./norm(cross(y_c,alpha));
y_b = cross(beta,x_b)./norm(cross(beta,x_b));
z_b = cross(x_b,y_b);

R = [x_b y_b z_b];

% Compute c
c = z_b' * (a_c + g + kd.*v_c);
c_cmd = c - kh*(v_c'*(x_b+y_b))^2;

% Compute c_dot
c_dot = z_b'*j_c + kd*z_b'*a_c;

% Compute w_x, w_y, w_z
w_x = (1/c)*(-y_b'*j_c - kd*y_b'*a_c);
w_y = (1/c)*(x_b'*j_c + kd*x_b'*a_c);
w_z = (1/norm(cross(y_c,z_b))) * (sigma_d2(4,1)*x_c'*x_b + w_y*y_c'*z_b);

w = [ w_x ; w_y ; w_z];

% Compute eta
w_acc = [ 0.0 -w_z  w_y ;...
          w_z  0.0 -w_x ;...
         -w_y  w_x  0.0];

eta = R*(w_acc*w_acc*D + D*w_acc*w_acc + 2*w_acc*D*w_acc')*R'*v_c +...
    2*R*(w_acc*D + D*w_acc')*R'*a_c +...
    R*D*R'*j_c;

% Find w_dot_x, w_dot_y, w_dot_z
w_dot_x = (1/c)*(-y_b'*s_c - 2*c_dot*w_x + c*w_y*w_z - y_b'*eta);
w_dot_y = (1/c)*( x_b'*s_c - 2*c_dot*w_y - c*w_x*w_z + x_b'*eta);
w_dot_z = (1/norm(cross(y_c,z_b))) * (w_dot_y*y_c'*z_b +...
           sigma_d2(4,1)*x_c'*x_b + ...
           2*sigma_d1(4,1)*w_z*x_c'*y_b - 2*sigma_d1(4,1)*w_y*x_c'*z_b -...
           w_x*w_y*y_c'*y_b - w_x*w_z*y_c'*z_b);

w_dot = [w_dot_x ; w_dot_y ; w_dot_z];

% Compute tau
tau_cmd = I*w_dot + A*R'*v_c + B*w + cross(w,I*w) + tau_g;

u_cmd = [m*c_cmd ; tau_cmd];

end

