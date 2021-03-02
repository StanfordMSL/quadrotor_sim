function log = logger_init(tf,N_act,N_est,N_fmu,x_init,model)

% Unpack some useful stuff
dt_act = model.dt_act;
dt_fmu = model.dt_fmu;
dt_est = model.dt_est;

% Timestamped Actual Pose Array
log.t_act = 0:dt_act:tf;
log.x_act = zeros(13,N_act);
log.x_act(:,1) = x_init;

% Estimator Data
log.t_est  = 0:dt_est:tf;
log.x_est = zeros(13,N_est);
log.x_est(:,1) = x_init;

% FMU Data
log.t_fmu  = 0:dt_fmu:tf;
log.u_w    = zeros(4,N_fmu-1);     % Wrench Input
log.u_br   = zeros(4,N_fmu-1);     % Body Rate Controller
log.u_m    = zeros(4,N_fmu-1);     % Motor Input (propellers)

% Capture time
log.t_capture = 999;
