function [sv, yukf, initial_bb, camera] = yolo_ukf_init(flight, t_arr)
    %%% PARAMS %%%
    yukf.prms.b_use_control = false;  % whether to use the control in our estimate
    %%%% OPTIONS FOR SENSOR %%%%%%%%%%%%%%%%%%%%%%%%
    % Option 1 %%%%%%%   z = [row, col, width, height, angle]
    yukf.prms.b_angled_bounding_box = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 2 %%%%%%%   z = [state]
    yukf.prms.b_measure_everything = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 3  %%%%%%%   z = [[row, col, width, height, <extra1>, <extra2>, ...]
    yukf.prms.b_measure_aspect_ratio = true; % when not angled, this will include a 5th value (ratio of height to width of bounding box)
    % ___extra A
    yukf.prms.b_measure_yaw = true; % adds the "true" yaw measurement as output of the sensor
    yukf.prms.b_enforce_0_yaw = true; % this overwrites any dynamics / incorrect update to keep yaw at 0
    % ___extra A
    yukf.prms.b_measure_pitch = false;
    % ___extra A
    yukf.prms.b_measure_roll = false;
    % ___extra B
    yukf.prms.b_measure_x = false;
    % ___extra C
    yukf.prms.b_measure_quat = false;
    % ___extra D
    yukf.prms.b_measure_pos = false;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    initial_bb = init_quad_bounding_box();
    camera = init_camera();

%     rel_state = get_relative_state(flight.x_act(:, 1), get_camera_state_vec(camera));

    yukf.dt = t_arr(2) - t_arr(1);
    
    yukf.mu = flight.x_act(:, 1);% + [1;2;-1;zeros(length(flight.x_act(:, 1)) - 3, 1)];
    yukf.mu_prev = yukf.mu;
    dim = length(yukf.mu);
    
    % these values are used for stepping along sigma directions
    dp = 0.1; % [m]
    dv = 0.05; % [m/s]
    dq = 0.001; % hard to say... steps of the vector portion of the quaternion - do this differently??
    dw = 0.005; % [rad/s]
    yukf.sigma = diag([dp, dp, dp, dv, dv, dv, dq, dq, dq, dw, dw, 0.0001]);
    yukf.prms.alpha = 1; % scaling param - how far sig. points are from mean
    yukf.prms.kappa = 2; % scaling param - how far sig. points are from mean
    yukf.prms.beta = 2; % optimal choice according to prob rob
    yukf.prms.lambda = yukf.prms.alpha^2*(dim + yukf.prms.kappa) - dim;
    yukf.prms.meas_len = length(predict_quad_bounding_box(yukf.mu, camera, initial_bb, yukf));

    yukf.prms.R = yukf.sigma/10;  % process noise
    yukf.prms.Q = diag([0.02, 0.02, ones(1, yukf.prms.meas_len - 2)])*5; 
    
    yukf.w0_m = yukf.prms.lambda / (yukf.prms.lambda + dim);
    yukf.w0_c = yukf.w0_m + (1 - yukf.prms.alpha^2 + yukf.prms.beta);
    yukf.wi = 1 / (2*(yukf.prms.lambda + dim));
    
    
    % yukf save variable for later plotting
    sv.mu_hist = zeros(dim, length(flight.t_act)); sv.mu_hist(:, 1) = yukf.mu;
    sv.dyn_ol_hist = sv.mu_hist; sv.do_ind = 2;% open loop propagation of dynamics
    sv.dyn_ol_hist2 = sv.dyn_ol_hist;% open loop propagation of dynamics
    sv.mu_act = zeros(dim, length(flight.t_act)); sv.mu_act(:, 1) = flight.x_act(:, 1);
    sv.sig_hist = zeros(dim, dim, length(flight.t_act)); sv.sig_hist(:, :, 1) = yukf.sigma;
    sv.sig_trace_hist = zeros(length(flight.t_act), 1); sv.sig_trace_hist(1) = trace(yukf.sigma);
    sv.time_hist = zeros(length(flight.t_act), 1); sv.time_hist(1) = 0;
    sv.hist_mask = false(length(flight.t_act), 1); sv.hist_mask(1) = true;

    qm = complete_unit_quat(yukf.mu(7:9, 1));
    [yaw, pitch, roll] = quat2angle(qm(:)');
    sv.ypr_hist = zeros(3, length(flight.t_act)); sv.ypr_hist(:, 1) = [yaw; pitch; roll]*180/pi;

    qa = complete_unit_quat(flight.x_act(7:9, 1)); 
    [yaw, pitch, roll] = quat2angle(qa(:)');
    sv.ypr_act_hist = zeros(3, length(flight.t_act)); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll]*180/pi;

    sv.ang_err = zeros(length(flight.t_act), 1); sv.ang_err(1) = quat_dist(qa, qm);
    sv.ang = zeros(length(flight.t_act), 1); sv.ang(1) = 0;
    sv.ang_act = zeros(length(flight.t_act), 1); sv.ang_act(1) = 0;
end