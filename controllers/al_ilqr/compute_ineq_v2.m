function [c_con, c_con_x, c_con_u] = compute_ineq_v2(x_curr,u_curr,model,wp,n_con)   
    %% Initialize constraints
    c_con = zeros(n_con,1);
    c_con_x = zeros(n_con,13);
    c_con_u = zeros(n_con,4);

    %% Motor Constraints
%     A_m_min = -(1/model.kt_act(1,1)).*[model.m2w_inv];
%     b_m_min = model.motor_min.^2 .* ones(4,1);
%     A_m_max = (1/model.kt_act(1,1)).*[model.m2w_inv];
%     b_m_max = -model.motor_max.^2 .* ones(4,1);
%     
%     c_con(1:4,:) = A_m_min*u_curr + b_m_min;
%     c_con(5:8,:) = A_m_max*u_curr + b_m_max;
%     
%     c_con_u(1:8,:) = [A_m_min ; A_m_max];
    
    %% Position Constraints
    p_g1 = wp.map(:,1);
    p_g2 = wp.map(:,2);
    p_g4 = wp.map(:,4);
    
    p_12 = p_g2 - p_g1;
    p_14 = p_g4 - p_g1;
    
    n_g = cross(p_14,p_12);
    
    p_b = x_curr(1:3,1);
    v_b = x_curr(4:6,1);
    
    alpha_c = dot((p_g1-p_b),n_g)/dot(v_b,n_g);
    
    if alpha_c > 0 && alpha_c < model.dt_ctl
        r = p_b + alpha_c.*v_b;
        beta_c = dot((r - p_g1),p_14)/dot(p_14,p_14);
        gamma_c = dot((r - p_g1),p_12)/dot(p_12,p_12);
        
        c_con(9,1)   =  beta_c - 0.8;
        c_con(10,1)  = -beta_c + 0.2;
        c_con(11,1)  =  gamma_c - 0.8;
        c_con(12,1)  = -gamma_c + 0.2;
  
        c_con_x(9,:)  = ( 1/dot(p_14,p_14)).*[p_14' alpha_c.*p_14' zeros(1,7)];
        c_con_x(10,:) = (-1/dot(p_14,p_14)).*[p_14' alpha_c.*p_14' zeros(1,7)];
        c_con_x(11,:) = ( 1/dot(p_12,p_12)).*[p_12' alpha_c.*p_12' zeros(1,7)];
        c_con_x(12,:) = (-1/dot(p_12,p_12)).*[p_12' alpha_c.*p_12' zeros(1,7)];
%         c_con_x(9,:)  = [ 1/delta(2,1) 0 0  alpha_c/delta(2,1) zeros(1,9)];
%         c_con_x(10,:) = [-1/delta(2,1) 0 0 -alpha_c/delta(2,1) zeros(1,9)];
%         c_con_x(11,:) = [ 1/delta(3,1) 0 0  alpha_c/delta(3,1) zeros(1,9)];
%         c_con_x(12,:) = [-1/delta(3,1) 0 0 -alpha_c/delta(3,1) zeros(1,9)];
    end
end