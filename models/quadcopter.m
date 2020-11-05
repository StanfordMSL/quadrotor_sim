function x_upd = quadcopter(x_curr,u_curr,model,FT_ext,type)   
    % Unpack
    g = model.g;
    
    switch type
        case 'actual'
          dt = model.dt_act; 
          wt = model.W*randn(13,1);
          
          I = model.I_act;
          inv_I = model.inv_I_act;
          
          F_drag = -model.kd_act .* x_curr(4:6,1);
          m = model.m_act;
          
          f_m_flag = 'actual';
        case 'fmu_ideal'
          dt = model.dt_fmu;
          wt = zeros(13,1);
          
          I = model.I_est;
          inv_I = model.inv_I_est;
          
          F_drag = -model.kd_est .* x_curr(4:6,1);
          m = model.m_est;
          
          f_m_flag = 'sim';
        case 'fmu_noisy'
          dt = model.dt_fmu;
          wt = model.W*randn(13,1);
          
          I = model.I_est;
          inv_I = model.inv_I_est;
          
          F_drag = -model.kd_est .* x_curr(4:6,1);
          m = model.m_est;
          
          f_m_flag = 'sim';
    end
    
    pos   = x_curr(1:3,1);
    vel   = x_curr(4:6,1);
    quat  = x_curr(7:10,1);
    wx = x_curr(11,1);
    wy = x_curr(12,1);
    wz = x_curr(13,1);
    w_all = x_curr(11:13,1);
    
    % Unpack External Force/Torques
    F_ext = FT_ext(1:3,1);
    tau_ext = FT_ext(4:6,1);
    
    % Calculate Motor Forces
    f_m = motor_transform(u_curr,model,f_m_flag);
    
    % Compute the Linear and Angular Accelerations
    vel_dot   = lin_acc(x_curr,m,f_m,F_drag,F_ext,g,0);
    omega_dot = ang_acc(f_m,I,inv_I,w_all,tau_ext,model.m2w);
    
    % State Updates   
    x_upd(1:3,1)   = pos   + dt*vel;        % World Frame Pos XYZ
    x_upd(4:6,1)   = vel   + dt*vel_dot;    % World Frame Vel XYZ
    x_upd(11:13,1) = w_all + dt*omega_dot;  % Body Frame Angular Vel XYZ
    
    % Do Our Quaternion Business 
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0];
    
    q_hat = quat + 0.5*Omega*quat*dt;
    q_norm = q_hat./norm(q_hat); 
    x_upd(7:10,1) = q_norm;
    
    x_upd = x_upd + wt;
end