clear; clc; 
addpath(genpath(pwd));

% Trajectory Name
traj_name = 'square_grp';

% Model
model = model_init('v1.0.1');   
model.dt_fmu = 1/5;

% Tuning Parameters
v_c = 1.0;          % Assumed constant velocity
fmu_dt  = model.dt_fmu;     % Waypoint frequency
n_der = 15;         % Order of Basis Function (n_der-1)

% Objective and Map Constraints
map = map_init('default');        % Initialize objectives
obj = obj_init(traj_name);           % Initialize objectives

% Unpack Some Terms
N_wp    = size(obj.x,2);

% Trajectory time setup. We assume a constant velocity of around 1.0m/s in
% between waypoints to generate an estimated time.
t_wp = zeros(1,N_wp);
for k = 1:N_wp-1
    s_int = norm(obj.x(1:3,k+1) - obj.x(1:3,k));
    t_int = fmu_dt*round(s_int/(fmu_dt*0.7));
    
    if s_int == 0   % hover (default)
        t_wp(1,k+1) = 2;
    else
        t_wp(1,k+1) = t_wp(1,k) + t_int;
    end
end

% Convert objectives to flat outputs
[t_sigma, sigma, con_sigma] = obj2sigma(obj,map,t_wp,model,n_der);

% Solve the Piecewise QP
f_out = piecewise_QP(t_sigma,sigma,con_sigma,fmu_dt);

% Extract the VRB position and velocity
fout_vrb = squeeze(f_out(:,1,:));
fout_dot_vrb = squeeze(f_out(:,2,:));

% Drone Swarm Parameters
n_robo = 3;
R_vrb  = 0.75;

n_fr    = size(fout_vrb,2);
f_out   = zeros(4,n_fr,n_robo);
t_f_out = zeros(5,n_fr,n_robo);

tf = t_sigma(1,end);
t_out = 0:fmu_dt:tf;
% t_out = 2 + t_out;

step = 2*pi/(n_robo-1);
theta_off = pi/2;
for k = 1:n_robo
    if k == 1
        traj = fout_vrb;
    else
        theta = (k-2).*step + theta_off;
        x_offset = R_vrb.*sin(theta);
        y_offset = R_vrb.*cos(theta);
        z_offset = 0;
        psi_offset = 0;
    
        pos_offset = [x_offset ; y_offset ; z_offset ; psi_offset];
        traj = fout_vrb + pos_offset;
    end
    t_traj =  [t_out ; traj];
    
    f_out(:,:,k) = traj;
    t_f_out(:,:,k) = t_traj;

    drone_traj_name = ['../TrajBridge-PX4/src/bridge_px4/trajectories/',traj_name '_drone' num2str(k) '.csv'];
    dlmwrite(drone_traj_name, t_traj, 'delimiter', ',','newline', 'pc');
end
%%
fout_animation_plot(f_out,map,obj,'persp')
