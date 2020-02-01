function flight = logger_init(tf,wp,act_hz,low_hz)

act_dt = 1/act_hz;
low_dt = 1/low_hz;

% Timestamped Actual Pose Array
flight.t_act = 0:act_dt:tf;
flight.x_act = zeros(13,length(flight.t_act));
flight.x_act(:,1) = wp.x(:,1);

% Timestamped FC Pose Array based on lowest controller
flight.t_fc  = 0:low_dt:tf;
flight.x_fc = zeros(13,length(flight.t_fc));
flight.x_fc(:,1) = wp.x(:,1);

flight.sigma = zeros(13,13,length(flight.t_fc));

% Flight Motor Inputs
flight.m_cmd = zeros(4,length(flight.t_fc)-1);
flight.u = zeros(4,length(flight.t_fc)-1);
