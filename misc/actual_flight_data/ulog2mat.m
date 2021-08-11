function [T,X,U] = ulog2mat(filename)

%% Process Flight Data

address = ['misc/actual_flight_data/' filename '/' filename '_'];

% Extract Raw Data
raw_att_act = csvread([address 'vehicle_attitude_0.csv'],1,0);
raw_pos_act = csvread([address 'vehicle_local_position_0.csv'],1,0);
raw_br_act  = csvread([address 'vehicle_angular_velocity_0.csv'],1,0);
raw_br_sp   = csvread([address 'vehicle_rates_setpoint_0.csv'],1,0);

% Process Into Useful Terms (x and u)
R = [ 0 1 0 ; 1 0 0 ; 0 0 -1];      % NED to ENU transform
t0 = raw_pos_act(1,1);

N = size(raw_pos_act,1);
T = zeros(1,N);
X = zeros(13,N);
U = zeros(4,N);

for k = 1:N
    % Time
    T(1,k) = (raw_pos_act(k,1)-t0)/(1e6);
    
    % Pos/Vel
    X(1:3,:) = R*raw_pos_act(:,5:7)';
    X(4:6,:) = R*raw_pos_act(:,11:13)';

    % Search for Matching Time Index
    t_ms = raw_pos_act(k,1);
    [~,idx_q] = min( abs(t_ms-raw_att_act(:,1)) );
    [~,idx_br] = min( abs(t_ms-raw_br_act(:,1)) );
    [~,idx_u] = min( abs(t_ms-raw_br_sp(:,1)) );

    % Quaternion and Body Rate
    X(7:10,k)  = raw_att_act(idx_q,2:5)'; 
    X(11:13,k) = raw_br_act(idx_br,3:5)';
    
    % Normalized Thrust+Body Rate Input Input
    U(:,k) = [-raw_br_sp(idx_u,7)' ; R*raw_br_sp(idx_u,2:4)'];
end