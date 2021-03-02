function thrust = thrust_ctrl(x_now,f_out,model)     

Kp = 3;
Kv = 1;

% Unpack Some Stuff
m = model.m_est;  
g = [ 0 ; 0 ; model.g];
kDz = model.D_est(3,3);

p_now   = x_now(1:3,1);
v_now   = x_now(4:6,1);
q_now   = x_now(7:10,1);
q_now_c = quatconj(q_now');

z_b = quatrotate(q_now_c,[0 0 1])';

p_des = f_out(1:3,1);
v_des = f_out(1:3,2);
a_des = f_out(1:3,3);

% Calculate the errors
e_p = p_now - p_des;
e_v = v_now - v_des;

F_des = -Kp.*e_p - Kv.*e_v + m.*g + m.*a_des + kDz.*v_des;

thrust = dot(F_des,z_b);

end
        