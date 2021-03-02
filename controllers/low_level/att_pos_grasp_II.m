function u_now = att_pos_grasp_II(x_now,f_out,model)     

% Constants
K_p = 1;
K_v = 1;

z_w = [0 ; 0 ; 1];

% Unpack some stuff
m = model.m_est;
g = model.g;

% NEED TO FIX %%%%%%%%%%%%%%%%%%%%%%%%
kh = 0.00;	
A  = 0.00.*eye(3,3);	
B  = 0.00.*eye(3,3);	
tau_g = 0.000.*ones(3,1);

D  = zeros(3,3);
kDx = 0;
kDy = 0;
kDz = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_ref = f_out(1:3,1);
v_ref = f_out(1:3,2);
a_ref = f_out(1:3,3);

% Compute alpha and beta
alpha = a_c + g + kDx.*v_c;
beta  = a_c + g + kDy.*v_c;

% Compute x_c and y_c
x_c = [ cos(f_out(4,1)) ; sin(f_out(4,1)) ; 0];
y_c = [-sin(f_out(4,1)) ; cos(f_out(4,1)) ; 0];

% Compute x_b, y_b and z_b
x_b = cross(y_c,alpha)./norm(cross(y_c,alpha));
y_b = cross(beta,x_b)./norm(cross(beta,x_b));
z_b = cross(x_b,y_b);

R = [x_b y_b z_b];
p_now = x_now(1:3,1);
v_now = x_now(4:6,1);

% Calculations
a_rd = -R*D*R'*v_ref;

a_fb = -K_p.*(p_now-p_ref) - K_v.*(v_now-v_ref);

a_des = a_fb + a_ref - a_rd + g.*z_w;

% deses
z_bdes = a_des./norm(a_des);
x_bdes = cross(y_c,z_bdes)./norm(cross(y_c,z_bdes));
y_bdes = cross(z_bdes,x_bdes);

p_ref = x_now(4:6,1);


 vrrotvec(a,b)

% Calculate some more stuff
q = x_now(7:10,1);
q_s = quatconj(q');
z_b = quatrotate( q_s, [0 0 1] );

e_p = r - r_T;
e_v = p_ref - r_T_dot;

F_des = -K_p.*e_p - K_v.*e_v + m*g.*z_w + m*r_T_ddot;

u1 = dot(F_des,z_b);

z_b_des = F_des/norm(F_des);
end
        