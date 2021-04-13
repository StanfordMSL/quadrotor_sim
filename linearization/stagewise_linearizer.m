function [A,B] = stagewise_linearizer(x_bar,u_bar,dt)

    % Unpack
    q_w = x_bar(7,1);
    q_x = x_bar(8,1);
    q_y = x_bar(9,1);
    q_z = x_bar(10,1);
    v_x = x_bar(4,1);
    v_y = x_bar(5,1);
    v_z = x_bar(6,1);
    w_x = x_bar(11,1);
    w_y = x_bar(12,1);
    w_z = x_bar(13,1);
    
    u1 = u_bar(1,1);
    u2 = u_bar(2,1);
    u3 = u_bar(3,1);
    u4 = u_bar(4,1);
    
%     J_x = J_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z,w_x,w_y,w_z);
    J_x = J_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,w_x,w_y,w_z);
    J_u = J_u_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4);
    
    A = eye(13) + dt.*J_x;
    B = dt.*J_u;
end