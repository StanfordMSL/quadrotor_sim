function [sv, yukf, initial_bb, camera] = yolo_ukf_init(flight)

    initial_bb = init_quad_bounding_box();
    camera = init_camera();

    yukf.mu = flight.x_act(:, 1);
    yukf.mu_prev = yukf.mu;
    dims = length(yukf.mu);
    yukf.sigma = eye(dims) * 0.01;
    yukf.prms.alpha = 1; % scaling param - how far sig. points are from mean
    yukf.prms.kappa = 2; % scaling param - how far sig. points are from mean
    yukf.prms.beta = 2; % optimal choice according to prob rob
    yukf.prms.lambda = yukf.prms.alpha^2*(dims + yukf.prms.kappa) - dims;
    yukf.prms.meas_len = length(predict_quad_bounding_box(yukf.mu, camera, initial_bb));

    yukf.prms.R = eye(dims) * 0.01;
    yukf.prms.Q = eye(yukf.prms.meas_len) * .1;

    % yukf save variable for later plotting
    sv.mu_hist = zeros(dims, length(flight.t_act)); sv.mu_hist(:, 1) = yukf.mu;
    sv.sig_hist = zeros(dims, dims, length(flight.t_act)); sv.sig_hist(:, :, 1) = yukf.sigma;
    sv.sig_trace_hist = zeros(length(flight.t_act), 1); sv.sig_trace_hist(1) = trace(yukf.sigma);
    sv.time_hist = zeros(length(flight.t_act), 1); sv.time_hist(1) = 0;
    sv.hist_mask = false(length(flight.t_act), 1); sv.hist_mask(1) = true;

    qm = complete_unit_quat(yukf.mu(7:9, 1));
    [yaw, pitch, roll] = quat2angle(qm(:)');
    sv.ypr_hist = zeros(3, length(flight.t_act)); sv.ypr_hist(:, 1) = [yaw; pitch; roll];

    qa = complete_unit_quat(flight.x_act(7:9, 1)); 
    [yaw, pitch, roll] = quat2angle(qa(:)');
    sv.ypr_act_hist = zeros(3, length(flight.t_act)); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll];

    sv.ang_err = zeros(length(flight.t_act), 1); sv.ang_err(1) = quat_dist(qa, qm);
    sv.ang = zeros(length(flight.t_act), 1); sv.ang(1) = 0;
    sv.ang_act = zeros(length(flight.t_act), 1); sv.ang_act(1) = 0;
end