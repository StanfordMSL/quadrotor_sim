function B = B_calc_wrench(mu_curr,model)

    dt = model.con_dt;
    
    q1 = mu_curr(7,1);
    q2 = mu_curr(8,1);
    q3 = mu_curr(9,1);
    q0 = sqrt(1 - q1^2 - q2^2 - q3^2);
    
    B = zeros(12,4);
    
    vel_coeff = (dt/model.m_est);
    vel_vect  = 2.*[(q1*q3+q2*q0) ; (q2*q3-q1*q0) ; (q3^2+q0^2+0.5)];
    B(4:6,1) = vel_coeff.*vel_vect;
    
    omega_coeff = dt*model.inv_I_est;
    B(10:12,:)   = [zeros(3,1) omega_coeff];
 end