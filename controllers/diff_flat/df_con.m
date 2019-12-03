function u_wrench = df_con(f_out,model,angle_axis)

% Unpack Items
g = model.g;
m = model.m_act;
I = model.I_act;

sigma    = f_out(:,1);
sigma_d1 = f_out(:,2);
sigma_d2 = f_out(:,3);
sigma_d3 = f_out(:,4);
sigma_d4 = f_out(:,5);

% Determine z_B
t = sigma_d2(1:3,1) + [0 0 g]';
u1 = m*norm(t);
z_B = t./norm(t);

% Generate intermediate forward axis
switch angle_axis
    case 'pitch'
        x_C = [cos(sigma(4,1)) 0 -sin(sigma(4,1))]';
    case 'yaw'
        x_C = [cos(sigma(4,1)) sin(sigma(4,1))  0]';
end

% Find y_B and x_B
vec1 = cross(z_B,x_C);
y_B = vec1./norm(vec1);
x_B = cross(y_B,z_B);

% Find h_omega
a_d1 = sigma_d3(1:3,1);
u_d1 = dot(z_B,m.*a_d1);
h_omega = (1./u1).*(m.*a_d1-u_d1.*z_B);

% Find omega
p = -dot(h_omega,y_B);
q = dot(h_omega,x_B);
r = sigma_d1(4,1);
omega = [p ; q ; r];

vec1 = cross(omega,omega);
vec2 = cross(vec1,u1.*z_B);
a_d2 = sigma_d4(1:3,1);
u_d2 = dot(z_B,m.*a_d2)-dot(z_B,vec2);
h_alpha = (m.*a_d2 - u_d2.*z_B - 2.*cross(omega,u_d1.*z_B)-vec2)./u1;
p_dot = -dot(h_alpha,y_B);
q_dot = dot(h_alpha,x_B);
r_dot =  sigma_d2(4,1);
alpha = [p_dot ; q_dot ; r_dot];
% acc_des(4:6,1) = alpha;

u_wrench = zeros(4,1);
u_wrench(1,1)   = u1;
u_wrench(2:4,1) = I*alpha + cross(omega,I*omega);
end

