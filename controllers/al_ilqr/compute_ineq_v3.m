function [c_con, c_con_x, c_con_u] = compute_ineq_v3(x_curr,u_curr,model,wp,n_con)   
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
    q_b = x_curr(7:10,1);
    omega_b = x_curr(11:13,1);
    
    l_arm = model.L_est;
    bRw = quat2rotm(q_b');
    p_arm = bRw*[0.00  l_arm  0.00]';    % converted  to world frame
    s_arm = bRw*[0.00 -l_arm  0.00]';    % converted  to world frame
    
    alpha_c = dot((p_g1-p_b),n_g)/dot(v_b,n_g);
    
    if alpha_c > 0 && alpha_c < model.dt_ctl
        r_p = (p_b + p_arm)+ alpha_c.*(v_b + cross(omega_b,p_arm));
        r_s = (p_b + s_arm)+ alpha_c.*(v_b + cross(omega_b,s_arm));

        beta_c_p = dot((r_p - p_g1),p_14)/dot(p_14,p_14);
        gamma_c_p = dot((r_p - p_g1),p_12)/dot(p_12,p_12);
        beta_c_s = dot((r_s - p_g1),p_14)/dot(p_14,p_14);
        gamma_c_s = dot((r_s - p_g1),p_12)/dot(p_12,p_12);
        
        % Port
        upp = 0.8;
        low = 0.2;
        c_con(9,1)   =  beta_c_p - upp;
        c_con(10,1)  = -beta_c_p + low;
        c_con(11,1)  =  gamma_c_p - upp;
        c_con(12,1)  = -gamma_c_p + low;
        % Starboard
        c_con(13,1)  =  beta_c_s - upp;
        c_con(14,1)  = -beta_c_s + low;
        c_con(15,1)  =  gamma_c_s - upp;
        c_con(16,1)  = -gamma_c_s + low;
  
        c_14 = 1/dot(p_14,p_14);
        c_12 = 1/dot(p_12,p_12);
        pos_14_dx = c_14.*[p_14' alpha_c.*p_14'];
        pos_12_dx = c_12.*[p_12' alpha_c.*p_12'];
        q_arm = l_arm.*[-2*q_b(4,1)  2*q_b(3,1)  2*q_b(2,1) -2*q_b(4,1);...
                       4*q_b(1,1)      0       4*q_b(3,1)     0      ;...
                       2*q_b(2,1)  2*q_b(1,1)  2*q_b(4,1) -2*q_b(3,1)];
        
        quat_14_dx = zeros(1,4);
        quat_12_dx = zeros(1,4);
        for k = 1:4
            feed = q_arm(:,k) + alpha_c.*cross(omega_b,q_arm(:,k));

            quat_14_dx(1,k) = dot(feed,p_14);
            quat_12_dx(1,k) = dot(feed,p_12);
        end
        
        omega_14_dx = zeros(1,3);
        omega_12_dx = zeros(1,3);
        I_split = eye(3);
        for k = 1:3
            inner = alpha_c.*cross(I_split(:,k),p_arm);
            
            omega_14_dx(1,k) = dot(inner,p_14);
            omega_12_dx(1,k) = dot(inner,p_12);
        end

        
        % Port
        c_con_x(9,:)  = [ pos_14_dx  quat_14_dx  omega_14_dx];  % p14
        c_con_x(10,:) = [-pos_14_dx -quat_14_dx -omega_14_dx];  % p14
        c_con_x(11,:) = [ pos_12_dx  quat_12_dx  omega_12_dx];  % p12
        c_con_x(12,:) = [-pos_12_dx -quat_12_dx -omega_12_dx];  % p12
        
        % Starboard
        c_con_x(13,:) = [ pos_14_dx -quat_14_dx  omega_14_dx];  % p14
        c_con_x(14,:) = [-pos_14_dx  quat_14_dx -omega_14_dx];  % p14
        c_con_x(15,:) = [ pos_12_dx -quat_12_dx  omega_12_dx];  % p12
        c_con_x(16,:) = [-pos_12_dx  quat_12_dx -omega_12_dx];  % p12
    end
end