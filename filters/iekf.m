function [mu_new, sigma_new] = iekf(mu_curr,sigma_curr,u,y,time,model)
    % y1  = x_camera
    % y2  = y_camera
    % y3  = z_camera
    % y4  = z_baro
    % y5  = theta_acc     
    % y6  = phi_acc
    % y7  = psi_acc
    % y8  = theta_mag
    % y9  = phi_mag
    % y10 = psi_mag
    % y11 = omega_x_gyro
    % y12 = omega_y_gyro
    % y13 = omega_z_gyro
    
    % x1  = x
    % x2  = y
    % x3  = z
    % x4  = x_dot
    % x5  = y_dot     
    % x6  = z_dot
    % x7  = q0
    % x8  = q1
    % x9  = q2
    % x10 = q3
    % x11 = omega_x
    % x12 = omega_y
    % x13 = omega_z
    
    mu_ekf_pred = zeros(13,1);
    % Build A and C Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    A = A_calc(mu_curr,u,model,time);
    C = C_calc(mu_curr,model);
    
    % Predict %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Predict Positions
    mu_ekf_pred(1:3,1) = mu_curr(1:3,1) + time.fc_dt*mu_curr(4:6,1);
    
    % Predict Velocities
    vel_dot = lin_acc(mu_curr,u,model,[0 0 0]',0,1);
    mu_ekf_pred(4:6) = mu_curr(4:6,1) + time.fc_dt*vel_dot;
    
    % Predict Quaternions
    % omegas
    wx = mu_curr(11,1);
    wy = mu_curr(12,1);
    wz = mu_curr(13,1);
    
    % Setup Some Useful Stuff for Pred %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0];
         
    q_hat = mu_curr(7:10,1) + 0.5*Omega*mu_curr(7:10,1)*time.fc_dt;
    mu_ekf_pred(7:10,1) = q_hat./norm(q_hat);
    
    % Predict Angular Velocities
    omega_dot = ang_acc(u,mu_curr(11:13),model,[0 0 0]',1);
    mu_ekf_pred(11:13,1) = mu_curr(11:13,1) + omega_dot*time.fc_dt;
    
    % Predict the Covariance
    sig_ekf_pred = A*sigma_curr*A'+model.Q_fil;

    % Update 
    mu_itr = mu_ekf_pred;
    diff = 10;
    itr_step = 0;
    itr_break = 0;
    
    while (diff > 1e-9)
        mu_itr_old = mu_itr;
        
        C = C_calc(mu_itr,model);
        g_iekf = sensors(time,mu_itr,u,model,1);      

        K = sig_ekf_pred*C'*inv(C*sig_ekf_pred*C'+model.R_fil);
        mu_itr = mu_ekf_pred + K*(y-g_iekf-C*(mu_ekf_pred-mu_itr));
        mu_itr(7:10,1) = mu_itr(7:10,1)./norm(mu_itr(7:10,1));

        diff = norm(mu_itr - mu_itr_old);
        
        if (diff > 10) || (itr_step > 3000)
            itr_break = 1;
            break            
        end
        
        itr_step = itr_step + 1;
    end    
    
    if itr_break == 0
        mu_new    = mu_itr;
%         mu_new(7:10,1) = mu_new(7:10,1)./norm(mu_new(7:10,1));
        sigma_new = (eye(13,13)-K*C)*sig_ekf_pred;
    else
        g_ekf = sensors(time,mu_ekf_pred,u,model,1);
        K = sig_ekf_pred*C'*inv(C*sig_ekf_pred*C'+model.R_fil);

        mu_new    = mu_ekf_pred  + K*(y-g_ekf);
        mu_new(7:10,1) = mu_new(7:10,1)./norm(mu_new(7:10,1));
        sigma_new = sig_ekf_pred - K*C*sig_ekf_pred;
        disp("NO CONVERGENCE. Switching to predict value.");
    end
end