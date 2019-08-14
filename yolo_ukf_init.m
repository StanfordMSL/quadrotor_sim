function [sv,initial_bb,camera,qtmp,qm,ukf_prms,mu_curr,mu_prev,sig_curr] = yolo_ukf_init(flight)

initial_bb = init_quad_bounding_box();
camera = init_camera();

mu_curr = flight.x_act(:, 1);
mu_prev = mu_curr;
dims = length(mu_curr);
sig_curr = eye(dims) * 0.01;
ukf_prms.alpha = 1; % scaling param - how far sig. points are from mean
ukf_prms.kappa = 2; % scaling param - how far sig. points are from mean
ukf_prms.beta = 2; % optimal choice according to prob rob
ukf_prms.lambda = ukf_prms.alpha^2*(dims + ukf_prms.kappa) - dims;
ukf_prms.meas_len = length(predict_quad_bounding_box(mu_curr, camera, initial_bb));

ukf_prms.R = eye(dims) * 0.01;
ukf_prms.Q = eye(ukf_prms.meas_len) * .1;

% yukf save variable for later plotting
sv.mu_hist = zeros(dims, length(flight.t_act)); sv.mu_hist(:, 1) = mu_curr;
sv.sig_hist = zeros(dims, dims, length(flight.t_act)); sv.sig_hist(:, :, 1) = sig_curr;
sv.sig_trace_hist = zeros(length(flight.t_act), 1); sv.sig_trace_hist(1) = trace(sig_curr);
sv.time_hist = zeros(length(flight.t_act), 1); sv.time_hist(1) = 0;
sv.hist_mask = false(length(flight.t_act), 1); sv.hist_mask(1) = true;

qtmp = mu_curr(7:9, 1);
qm = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qm(:)');
sv.ypr_hist = zeros(3, length(flight.t_act)); sv.ypr_hist(:, 1) = [yaw; pitch; roll];

qtmp = flight.x_act(7:9 ,1); 
qa = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qa(:)');
sv.ypr_act_hist = zeros(3, length(flight.t_act)); sv.ypr_act_hist(:, 1) = [yaw; pitch; roll];

sv.ang_err = zeros(length(flight.t_act), 1); sv.ang_err(1) = quat_dist(qa, qm);
sv.ang = zeros(length(flight.t_act), 1); sv.ang(1) = 0;
sv.ang_act = zeros(length(flight.t_act), 1); sv.ang_act(1) = 0;

