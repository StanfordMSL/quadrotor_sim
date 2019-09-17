function yukf = yolo_ukf_init(num_dims, dt)
    yukf.dt = dt;
    
    %%% PARAMS %%%
    yukf.prms.b_use_control = false;  % whether to use the control in our estimate
    yukf.b_offset_at_t0 = false;  % whether to add noise to the initial starting location
    dyn_weight = 1;
    %%%% OPTIONS FOR SENSOR %%%%%%%%%%%%%%%%%%%%%%%%
    yukf.prms.b_predicted_bb = true; % true means sensing data comes from predict_quad_bounding_box() instead of from actual yolo data
    yukf.prms.b_filter_data = true; % decide if we want to filter data output from yolo (only has an effect if we are using real data, i.e. if b_predicted_bb = false)
    % Option 1 %%%%%%%   z = [row, col, width, height, angle]
    yukf.prms.b_angled_bounding_box = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 2 %%%%%%%   z = [state]
    yukf.prms.b_measure_everything = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 3  %%%%%%%   z = [[row, col, width, height, <extra1>, <extra2>, ...]
    yukf.prms.b_measure_aspect_ratio = false; % when not angled, this will include a 5th value (ratio of height to width of bounding box)
    % ___extra A
    yukf.prms.b_measure_yaw = true; % adds the "true" yaw measurement as output of the sensor
    yukf.prms.b_enforce_yaw = true; % this overwrites any dynamics / incorrect update to keep yaw at ground truth value
    yukf.prms.b_enforce_0_yaw = false; % this overwrites any dynamics / incorrect update to keep yaw at 0
    % ___extra A
    yukf.prms.b_measure_pitch = false;
    yukf.prms.b_enforce_pitch= false; % this overwrites any dynamics / incorrect update to keep pitch at ground truth value
    % ___extra A
    yukf.prms.b_measure_roll = false;
    yukf.prms.b_enforce_roll = false; % this overwrites any dynamics / incorrect update to keep roll at ground truth value
    % ___extra B
    yukf.prms.b_measure_x = false;
    % ___extra C
    yukf.prms.b_measure_quat = false;
    % ___extra D
    yukf.prms.b_measure_pos = false;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    yukf.mu = zeros(num_dims, 1);
    yukf.mu_prev = yukf.mu;
    dim = length(yukf.mu);
    
    % these values are used for stepping along sigma directions
    dp = 0.1; % [m]
    dv = 0.005; % [m/s]
    dq = 0.001; % hard to say... steps of the vector portion of the quaternion - do this differently??
    dw = 0.0005; % [rad/s]
    yukf.sigma = diag([dp, dp, dp, dv, dv, dv, dq, dq, dq, dw, dw, dw]);
    yukf.prms.alpha = 1; % scaling param - how far sig. points are from mean
    yukf.prms.kappa = 2; % scaling param - how far sig. points are from mean
    yukf.prms.beta = 2; % optimal choice according to prob rob
    yukf.prms.lambda = yukf.prms.alpha^2*(dim + yukf.prms.kappa) - dim;
    fake_cam.tf_cam_w = eye(4); fake_cam.K = eye(3);
    yukf.prms.meas_len = length(predict_quad_bounding_box(yukf.mu, fake_cam, rand(size(init_quad_bounding_box())), yukf));

    yukf.prms.R = yukf.sigma/10;  % process noise
    yukf.prms.Q = diag([0.02, 0.02, ones(1, yukf.prms.meas_len - 2)])*5*dyn_weight; 
    
    yukf.w0_m = yukf.prms.lambda / (yukf.prms.lambda + dim);
    yukf.w0_c = yukf.w0_m + (1 - yukf.prms.alpha^2 + yukf.prms.beta);
    yukf.wi = 1 / (2*(yukf.prms.lambda + dim));
end