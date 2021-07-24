function [con,con_x,con_u] = fp_con_upd(xk,uk,up,plane,p_box)

con = zeros(24,1);
con_x = zeros(24,10);
con_u = zeros(24,4);

con(1:8,1)    = motor_con(xk,uk,up);
con_x(1:8,:)  = motor_con_x(xk,uk,up);
con_u(1:8,:)  = motor_con_u(xk,uk,up);

dist = plane_dist(plane,xk(1:3));
if dist < 0.1
    con(9:24,1)   = gate_con(xk,uk,p_box);
    con_x(9:24,:) = gate_con_x(xk,uk,p_box);
    con_u(9:24,:) = gate_con_u(xk,p_box);
end

