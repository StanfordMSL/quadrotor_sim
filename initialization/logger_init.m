function log = logger_init(wp,dt_act,dt_fbc)

% Timestamped Actual Pose Array
log.t_act = 0:dt_act:wp.tf;
log.x_act = zeros(13,length(log.t_act));
log.x_act(:,1) = wp.x(:,1);

% Timestamped FC Pose Array based on lowest controller
log.t_fc  = 0:dt_fbc:wp.tf;
log.x_fc = zeros(13,length(log.t_fc));
log.x_fc(:,1) = wp.x(:,1);

log.sigma = zeros(13,13,length(log.t_fc));

% Flight Motor Inputs
log.m_cmd = zeros(4,length(log.t_fc)-1);
log.u = zeros(4,length(log.t_fc)-1);
