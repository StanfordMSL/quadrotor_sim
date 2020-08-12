function log = logger_init(tf,N_act,N_fmu,x_init,model)

% Unpack some useful stuff
dt_act = model.dt_act;
dt_fmu = model.dt_fmu;

% Timestamped Actual Pose Array
log.t_act = 0:dt_act:tf;
log.x_act = zeros(13,N_act);
log.x_act(:,1) = x_init;

% Timestamped FC Pose Array based on lowest controller
log.t_fmu  = 0:dt_fmu:tf;
log.x_fmu = zeros(13,N_fmu);
log.x_fmu(:,1) = x_init;

% Flight Wrench Inputs
log.u_fmu = zeros(4,N_fmu-1);

% Capture time
log.t_capture = 999;
