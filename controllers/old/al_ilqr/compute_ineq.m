function [c_con, c_con_x, c_con_u] = compute_ineq(x_curr,u,model,n_con)
    
    % Unpack some data
    pos = x_curr(1:3,1);
    
    % Motor Constraints
    A_m_min = -(1/model.kt_act(1,1)).*[model.m2w_inv];
    b_m_min = model.motor_min.^2 .* ones(4,1);
    A_m_max = (1/model.kt_act(1,1)).*[model.m2w_inv];
    b_m_max = -model.motor_max.^2 .* ones(4,1);
    
    % Position Constraint
    pos_wall = 0.0;
    a = 0.001;
    term = (pos(1,1)-pos_wall)/a;
    
    dirac = exp(-term.^2);
    
    A_pos = pos(3,1)-1.0;
    b_pos = 0.05;
    
    % Compute our constraint values
    c_con = zeros(n_con,1);
    c_con_x = zeros(n_con,13);
    c_con_u = zeros(n_con,4);
    
    c_con(1:4,:) = A_m_min*u + b_m_min;
    c_con(5:8,:) = A_m_max*u + b_m_max;
    c_con(9,:)   = dirac.*(-A_pos + b_pos);
    
    c_con_x(9,1)   = (2./a).*term.*dirac.*(pos(1,1)-pos_wall-0.5);
    c_con_x(9,3)   = -dirac; 
    c_con_u(1:8,:) = [A_m_min ; A_m_max];
end