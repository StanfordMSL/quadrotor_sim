function B = B_calc(mu_curr,model)
    
    dt    = model.dt_fmu;

    % Quaternions
    q0 = mu_curr(7,1);
    q1 = mu_curr(8,1);
    q2 = mu_curr(9,1);
    q3 = mu_curr(10,1);
    
    % B for Position
    B_pos = zeros(3,4);
    
    % B for Velocity
    vel_vect  = 2.*[(q1*q3+q2*q0) ; (q2*q3-q1*q0) ; (q3^2+q0^2+0.5)];    
    B_vel = [(dt/model.m_est).*vel_vect zeros(3,3)];
    
    % B for Quaternions
    B_quat = zeros(4,4); 
    
    % B for Omegas
    B_omega   = [zeros(3,1) dt*model.inv_I_est]; 
    
    % Combine the Bs
    B = [B_pos ; B_vel ; B_quat ; B_omega];
 end