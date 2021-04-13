function x_upd = quadcopter(x_curr,u_curr,model,FT_ext,type)   
    % Unpack    
    F_ext_x   = FT_ext(1,1);
    F_ext_y   = FT_ext(2,1);
    F_ext_z   = FT_ext(3,1);
    tau_ext_x = FT_ext(4,1);
    tau_ext_y = FT_ext(5,1);
    tau_ext_z = FT_ext(6,1);
    
    pos   = x_curr(1:3,1);
    
    vel   = x_curr(4:6,1);
    v_x   = vel(1,1);
    v_y   = vel(2,1);
    v_z   = vel(3,1);
    
    quat  = x_curr(7:10,1);
    q_w   = quat(1,1);
    q_x   = quat(2,1);
    q_y   = quat(3,1);
    q_z   = quat(4,1);
    
    w   = x_curr(11:13,1);
    w_x = w(1,1);
    w_y = w(2,1);
    w_z = w(3,1);
    
    u1 = u_curr(1,1);
    u2 = u_curr(2,1);
    u3 = u_curr(3,1);
    u4 = u_curr(4,1);
    
    switch type
        case 'actual'
            dt = model.dt_act;
            wt = model.W*randn(13,1);
            
            disp('[quadcopter]: Limits on actual turned off.');
            for k = 1:4
                if u_curr(k,1) > model.motor_max
                    u_curr(k,1) = model.motor_max;
                    disp('[quadcopter]: Upper limit triggered.');
                elseif u_curr(k,1) < model.motor_min
                    u_curr(k,1) = model.motor_min;
                    disp('[quadcopter]: Lower limit triggered.');
                end
            end
            
%             vel_dot   = lin_acc_act(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z);
%             omega_dot = ang_acc_act(q_w,q_x,q_y,q_z,tau_ext_x,tau_ext_y,tau_ext_z,u1,u2,u3,u4,v_x,v_y,v_z,w_x,w_y,w_z);
            vel_dot   = lin_acc_act(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4);
            omega_dot = ang_acc_act(tau_ext_x,tau_ext_y,tau_ext_z,u1,u2,u3,u4,w_x,w_y,w_z);
        case 'fmu_ideal'
            dt = model.dt_fmu;
            wt = zeros(13,1);
            
%             vel_dot   = lin_acc_est(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z);
%             omega_dot = ang_acc_est(q_w,q_x,q_y,q_z,tau_ext_x,tau_ext_y,tau_ext_z,u1,u2,u3,u4,v_x,v_y,v_z,w_x,w_y,w_z);
            vel_dot   = lin_acc_est(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4);
            omega_dot = ang_acc_est(tau_ext_x,tau_ext_y,tau_ext_z,u1,u2,u3,u4,w_x,w_y,w_z);
        case 'fmu_noisy'
            dt = model.dt_fmu;
            wt = model.W*randn(13,1);
            
            vel_dot   = lin_acc_est(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4);
            omega_dot = ang_acc_est(tau_ext_x,tau_ext_y,tau_ext_z,u1,u2,u3,u4,w_x,w_y,w_z);
   end

    % Do Our Quaternion Business 
    % NOTE: We are using the Aerospace toolbox definition of rotation,
    % which is passive. Therefore the rotation is going to be quat(W->B).
          
    Omega  = [  0 -w_x -w_y -w_z ;...
              w_x    0  w_z -w_y ;...
              w_y -w_z    0  w_x ;...
              w_z  w_y -w_x    0];
    
    q_hat = quat + 0.5*Omega*quat*dt;
    q_norm = q_hat./norm(q_hat); 
    
    % State Updates   
    x_upd(1:3,1)   = pos   + dt*vel;        % World Frame Pos XYZ
    x_upd(4:6,1)   = vel   + dt*vel_dot;    % World Frame Vel XYZ
    x_upd(7:10,1)  = q_norm;
    x_upd(11:13,1) = w     + dt*omega_dot;      % Body Frame Angular Vel XYZ
    
    x_upd = x_upd + wt;
end