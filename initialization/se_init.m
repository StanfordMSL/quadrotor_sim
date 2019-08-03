function est = se_init(mode,hz)

est.mode = mode;
est.Q = 0.1*eye(13);

% Assume all flights are in flightroom
mag_mag = 55.66;          % Magnetometer Magnitude in Flight Room (muT)

% Basic Quadrotor has a Baro,Accelerometer,Magnetometer,Gyroscope
bar_model = (1.2e-1)*(1/sqrt(hz)).*ones(1,1);
acc_model  = (2.94e-3)*(1/sqrt(hz)).*ones(3,1);
mag_model  = (0.042/mag_mag)*(1/sqrt(hz)).*ones(3,1);
gyr_model = (1.75e-4)*(1/sqrt(hz)).*ones(3,1);
imu_model = [bar_model ; acc_model ; mag_model ; gyr_model];

switch mode
    case 'imu'
        sensor_model = imu_model;
    case 'mocap'
        pos_data = (20e-9).*ones(3,1);
        quat_data = (20e-9).*ones(4,1);
        mocap_model = [pos_data ; quat_data];
        
        sensor_model = [imu_model ; mocap_model];
    case 'vio'
        px_xy_data = (20e-9).*ones(3,1);
        px_z_data  = (20e-3).*ones(3,1);
        
        sensor_model = [imu_model ; px_xy_data ; px_z_data];
    case 'perfect'
        sensor_model = 1e-5.*imu_model;
end

est.R = diag(sensor_model);
est.length = length(sensor_model);
end
