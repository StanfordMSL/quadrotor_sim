function log = logger_init(tf,N_act,N_ses,N_fmu,traj,model)

% Unpack some useful stuff
dt_act = model.clock.dt_act;
dt_fmu = model.clock.dt_fmu;
dt_ses = model.clock.dt_ses;

% Timestamped Actual Pose Array
log.t_act = 0:dt_act:tf;
log.x_act = zeros(13,N_act);
log.x_act(:,1) = traj.x_bar(:,1);

% Estimator Data
log.t_ses  = 0:dt_ses:tf;
log.x_ses  = zeros(13,N_ses);
log.x_ses(:,1) = traj.x_bar(:,1);

log.sigma = zeros(13,13,N_ses);

log.z_ses = zeros(7,N_ses);

% Store Desired Trajectory
log.x_des = traj.x_bar;

% FMU Data
log.t_fmu  = 0:dt_fmu:tf;
log.x_fmu  = zeros(13,N_fmu);
log.u_br   = zeros(4,N_fmu-1);     % Body Rate Controller
log.u_mt   = zeros(4,N_fmu-1);     % Motor Input (propellers)

% Capture time
log.t_capture = 999;
