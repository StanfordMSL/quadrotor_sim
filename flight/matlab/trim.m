function traj_t = trim(traj,k_now)

% Trim trajectory down to relevant portion.
traj_t.x_bar = traj.x_bar(:,k_now:end);
traj_t.x_br  = traj.x_br(:,k_now:end);
traj_t.u_br  = traj.u_br(:,k_now:end);
traj_t.L_br  = traj.L_br(:,:,k_now:end);

traj_t.t_fmu = traj.t_fmu(:,k_now:end)-traj.t_fmu(:,k_now);
traj_t.hz = traj.hz;
traj_t.type = traj.type;

end