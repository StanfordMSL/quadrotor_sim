function [con,con_x,con_u] = motor_con(u_bar,kt_act,m2w_inv,motor_min,motor_max)

%     A_m_min = -(1/kt_act).*m2w_inv;
%     b_m_min = motor_min.^2 .* ones(4,1);
%     A_m_max = (1/kt_act).*m2w_inv;
%     b_m_max = -motor_max.^2 .* ones(4,1);
% 
%     con(1:4,1) = A_m_min*u_bar + b_m_min;
%     con(5:8,1) = A_m_max*u_bar + b_m_max;
% 
%     con_x = zeros(8,13);
%     con_u(1:8,:) = [A_m_min ; A_m_max];
    
    con   = -999.*ones(8,1);
    con_x = zeros(8,13);
    con_u = zeros(8,4);
end