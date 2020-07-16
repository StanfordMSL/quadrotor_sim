function B = B_calc_wrench(mu_curr,u_curr,model)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Inputs
    f_thr = u_curr(1,1);
    tau   = u_curr(2:4,1);
    dt    = u_curr(5,1);
    
    % Velocity
    vel = mu_curr(4:6,1);
    
    % Quaternions
    q0 = mu_curr(7,1);
    q1 = mu_curr(8,1);
    q2 = mu_curr(9,1);
    q3 = mu_curr(10,1);
    
    quat = mu_curr(7:10,1);
        
    % Omegas
    wx = mu_curr(11,1);
    wy = mu_curr(12,1);
    wz = mu_curr(13,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % B for Position
    B_pos = [zeros(3,4) vel];
    
    % B for Velocity
    vel_vect  = 2.*[(q1*q3+q2*q0) ; (q2*q3-q1*q0) ; (q3^2+q0^2+0.5)];
        
    F_grav   = [ 0 ; 0 ; -model.m_est*model.g];
    F_thrust = [ 0 ; 0 ; f_thr];
    F_drag   = -model.kd_est .* mu_curr(4:6,1);
    bRw = quat2rotm(quat');
    vel_dt_c = (1/model.m_est)*(F_grav + bRw*F_thrust + F_drag);
        
    B_vel = [(dt/model.m_est).*vel_vect zeros(3,3) vel_dt_c];
    
    % B for Quaternions
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0];
    quat_dt_c = 0.5.*Omega*quat; 
    B_quat = [zeros(4,4) quat_dt_c]; 
    
    % B for Omegas
    omega_dt_c = model.inv_I_est*tau;

    B_omega   = [zeros(3,1) dt*model.inv_I_est  omega_dt_c]; 
    
    % Combine the Bs
    B = [B_pos ; B_vel ; B_quat ; B_omega];
 end