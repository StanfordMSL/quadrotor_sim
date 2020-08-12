function [con,con_x,con_u] = gate_con(x_bar,pnts_gate,l_arm,dt_ctl)
    %% Position Constraints
    p_g1 = pnts_gate(:,1);
    p_g2 = pnts_gate(:,2);
    p_g4 = pnts_gate(:,4);
    
    p_12 = p_g2 - p_g1;
    p_14 = p_g4 - p_g1;
    
    n_g = cross(p_14,p_12);
    
    p_b = x_bar(1:3,1);
    v_b = x_bar(4:6,1);
    q_b = x_bar(7:10,1);
    omega_b = x_bar(11:13,1);
    
    bRw = quat2rotm(q_b');
    p_arm = bRw*[0.00  l_arm  0.00]';    % converted  to world frame
    s_arm = bRw*[0.00 -l_arm  0.00]';    % converted  to world frame
    
    if dot(v_b,n_g) == 0
        % never hitting
        con = zeros(8,1);
        con_x = zeros(8,13);
        con_u =  zeros(8,4);
    else
        alpha_c = dot((p_g1-p_b),n_g)/dot(v_b,n_g);

        if alpha_c > 0 && alpha_c < dt_ctl
            r_p = (p_b + p_arm)+ alpha_c.*(v_b + cross(omega_b,p_arm));
            r_s = (p_b + s_arm)+ alpha_c.*(v_b + cross(omega_b,s_arm));

            beta_c_p = dot((r_p - p_g1),p_14)/dot(p_14,p_14);
            gamma_c_p = dot((r_p - p_g1),p_12)/dot(p_12,p_12);
            beta_c_s = dot((r_s - p_g1),p_14)/dot(p_14,p_14);
            gamma_c_s = dot((r_s - p_g1),p_12)/dot(p_12,p_12);

            % Port
            upp = 0.8;
            low = 0.2;
            con(1,1)   =  beta_c_p - upp;
            con(2,1)  = -beta_c_p + low;
            con(3,1)  =  gamma_c_p - upp;
            con(4,1)  = -gamma_c_p + low;
            % Starboard
            con(5,1)  =  beta_c_s - upp;
            con(6,1)  = -beta_c_s + low;
            con(7,1)  =  gamma_c_s - upp;
            con(8,1)  = -gamma_c_s + low;

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
            con_x(1,:)  = [ pos_14_dx  quat_14_dx  omega_14_dx];  % p14
            con_x(2,:) = [-pos_14_dx -quat_14_dx -omega_14_dx];  % p14
            con_x(3,:) = [ pos_12_dx  quat_12_dx  omega_12_dx];  % p12
            con_x(4,:) = [-pos_12_dx -quat_12_dx -omega_12_dx];  % p12

            % Starboard
            con_x(5,:) = [ pos_14_dx -quat_14_dx  omega_14_dx];  % p14
            con_x(6,:) = [-pos_14_dx  quat_14_dx -omega_14_dx];  % p14
            con_x(7,:) = [ pos_12_dx -quat_12_dx  omega_12_dx];  % p12
            con_x(8,:) = [-pos_12_dx  quat_12_dx -omega_12_dx];  % p12

            con_u =  zeros(8,4);
        else
            con = zeros(8,1);

            con_x = zeros(8,13);

            con_u =  zeros(8,4);
        end
    end
        
end