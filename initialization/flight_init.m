function flight = flight_init(model,tf,wp)

act_dt = 1/model.act_hz;
con_dt = 1/model.con_hz;

% Timestamped Actual Pose Array
flight.t_act = 0:act_dt:tf;
flight.x_act = zeros(13,length(flight.t_act));
flight.x_act(:,1) = wp.x(:,1);

% Timestamped FC Pose Array with Cov
flight.t_fc  = 0:con_dt:tf;
flight.x_fc = zeros(13,length(flight.t_fc));
flight.x_fc(:,1) = wp.x(:,1);

flight.sigma = zeros(13,13,length(flight.t_fc));

% Flight Motor Inputs
flight.m_cmd = zeros(4,length(flight.t_fc)-1);
flight.m_cmd(:,1) = wrench2m_controller(model.hover_wrench,model);
flight.u = zeros(4,length(flight.t_fc)-1);
