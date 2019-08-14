function x_upd = quadcopter(x_curr,u_curr,model,FT_ext,type)   
    % Unpack
    switch type
        case 'actual'
          dt = model.act_dt;  
        case 'fc'
          dt = model.con_dt;
    end
    
    pos   = x_curr(1:3,1);
    vel   = x_curr(4:6,1);
    q  = x_curr(7:9,1);
    q0 = sqrt(1-q'*q);
    quat  = [q0 ; q];
    wx = x_curr(10,1);
    wy = x_curr(11,1);
    wz = x_curr(12,1);
    w_all = x_curr(10:12,1);

    % Generate Noise
    wt = model.Q*randn(6,1);
    
    % Unpack External Force/Torques
    F_ext = FT_ext(1:3,1);
    tau_ext = FT_ext(4:6,1);
    
    % Compute the Linear and Angular Accelerations
    vel_dot   = lin_acc(x_curr,u_curr,model,F_ext,0,'actual') + wt(1:3,1);
    omega_dot = ang_acc(u_curr,w_all,model,tau_ext,'actual') + wt(4:6,1);
    
    % State Updates   
    x_upd(1:3,1)   = pos   + dt*vel;        % World Frame Pos XYZ
    x_upd(4:6,1)   = vel   + dt*vel_dot;    % World Frame Vel XYZ
    x_upd(10:12,1) = w_all + dt*omega_dot;  % Body Frame Angular Vel XYZ
    
    % Do Our Quaternion Business 
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0];
    
    q_hat = quat + 0.5*Omega*quat*dt;
    q_norm = q_hat./norm(q_hat); 
    x_upd(7:9,1) = q_norm(2:4,1); 
end