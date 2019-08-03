function B = B_calc(mu_curr,u_curr,model)

    dt = model.fc_dt;
    
    q1 = mu_curr(7,1);
    q2 = mu_curr(8,1);
    q3 = mu_curr(9,1);
    q0 = sqrt(1 - q1^2 - q2^2 - q3^2);

    u1 = u_curr(1,1);
    u2 = u_curr(2,1);
    u3 = u_curr(3,1);
    u4 = u_curr(4,1);
    
    B = zeros(12,4);
    
    vel_coeff = (dt/model.m_est)*model.kt_est(1,1);
    vel_vect  = 2.*[(q1*q3+q2*q0) ; (q2*q3-q1*q0) ; (q3^2+q0^2+0.5)];
    
    for k = 1:4
        B(4:6,k) = vel_coeff*vel_vect*(2*u_curr(k,1));
    end
    
    omega_coeff = dt*model.inv_I_est;
    partial_omega_x = model.L_est*model.kt_est(1,1)*2*[-u1  u2  u3 -u4];
    partial_omega_y = model.L_est*model.kt_est(1,1)*2*[-u1  u2 -u3  u4];
    partial_omega_z = model.b_est*model.kt_est(1,1)*2*[-u1 -u2  u3  u4];
    B(10:12,:) = omega_coeff*[partial_omega_x ; partial_omega_y ; partial_omega_z];  
 end