function y = sensors(time,states,u_curr,model,type)
    %     % Sensor Frequency (TODO)
    %     imu_hz = 500;
    %     vis_hz = 30;
    
    % Build C Matrix
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

    % Generate Noise
    vt_cam = model.R_cam*randn(3,1);
    vt_imu = model.R_imu*randn(10,1);
    vt = [vt_cam ; vt_imu];
    
    % Calculate Some Useful Values
    bRw = quat2rotm(states(7:10,1)');
    wRb = bRw';

    % Generate Sensor Data   
    y_cam = states(1:3,1);
  
    y_baro = states(3,1);
    y_acc  = lin_acc(states,u_curr,model,[0 0 0]',1,0);    
    y_mag  = wRb*[0 1 0]';
    y_gyro = states(11:13,1);
    
    y_imu = [y_baro ; y_acc ; y_mag ; y_gyro ];

    if type == 1            % where 1 = no noise, 0 = with noise
        y = [y_cam ; y_imu];
    else
        y = [y_cam ; y_imu] + vt;
    end
end