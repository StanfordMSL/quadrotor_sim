function cost_param = cost_assembly(traj,wts_db,obj)

% Count
N_x = size(traj.x,1);
N_u = size(traj.u,1);
N_s = size(traj.x,2);

% Initialize the Cost Struct.
cost_param.Q = zeros(N_x,N_x,N_s);
cost_param.R = zeros(N_u,N_u,(N_s-1));

% We only care about our final point.
for k = 1:(N_s-1)
    cost_param.Q(:,:,k) = wts_db.Q_zero;
    cost_param.R(:,:,k) = wts_db.R_stnd;
end
cost_param.Q(:,:,end) = 10.*wts_db.Q_hover;
% cost_param.Q(:,:,end) = wts_db.Q_unif;

% Our objectives: Target waypoint and minimum energy.
cost_param.x_star = obj.wp_arr(:,end);
cost_param.u_star = zeros(4,1);