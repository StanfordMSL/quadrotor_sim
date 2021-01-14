function [con,con_x,con_u] = basis_con(alpha,r_G1C,r_BAS,r_arm,x_bar)    

% Unpack Some Useful Stuff
q1  = x_bar(7,1);
q2  = x_bar(8,1);
q3  = x_bar(9,1);
q4  = x_bar(10,1);
q_b = x_bar(7:10,1);
bRw = quat2rotm(q_b');
r_ARM = bRw*r_arm;

w1 = x_bar(4,1);
w2 = x_bar(5,1);
w3 = x_bar(6,1);
 
r_w1 = r_ARM(1,1);
r_w2 = r_ARM(2,1);
r_w3 = r_ARM(3,1);

omega_cpm = [ 0  -w3  w2 ;...
              w3   0 -w1 ;...
             -w2  w1  0 ];

r_w_cpm   = [   0  -r_w3  r_w2 ;...
              r_w3   0   -r_w1 ;...
             -r_w2  r_w1    0 ];

M_q1 = [ 4*q1 -2*q4  2*q3 ;...
         2*q4  4*q1 -2*q2 ;...
        -2*q3  2*q2  4*q1];
    
M_q2 = [ 4*q2  2*q3  2*q4 ;...
         2*q3    0  -2*q1 ;...
         2*q4  2*q1    0 ];

M_q3 = [   0   2*q2  2*q1 ;...
         2*q2  4*q3  2*q4 ;...
        -2*q1  2*q4    0 ];

M_q4 = [   0  -2*q1  2*q2 ;...
         2*q1    0   2*q3 ;...
         2*q2  2*q3  4*q4];

T_q = [M_q1*r_arm M_q2*r_arm M_q3*r_arm M_q4*r_arm];

% Initialize outputs
con   = zeros(2,1);
con_x = zeros(2,13);
con_u = zeros(2,4);

% Compute Gain
gain = dot(r_G1C,r_BAS)/dot(r_BAS,r_BAS);

con(1,:) =  gain - 0.9;
con(2,:) = -gain + 0.1;

% r_c'
p_comp = eye(3);
v_comp = alpha.*eye(3);
q_comp = T_q + alpha.*(omega_cpm * T_q);
omega_comp = -alpha.*r_w_cpm;

r_c_prime = [p_comp v_comp q_comp omega_comp];

con_x(1,:) =  (r_BAS'*r_c_prime)./dot(r_BAS,r_BAS);
con_x(2,:) = -(r_BAS'*r_c_prime)./dot(r_BAS,r_BAS);

end