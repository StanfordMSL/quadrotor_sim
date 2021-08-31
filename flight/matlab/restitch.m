function traj_o = restitch(traj,traj_t,k_now)

% Put trajectory back together
traj_o.x_bar = cat(2,traj.x_bar(:,1:k_now-1),traj_t.x_bar);
traj_o.x_br  = cat(2,traj.x_br(:,1:k_now-1),traj_t.x_br);
traj_o.u_br  = cat(2,traj.u_br(:,1:k_now-1),traj_t.u_br);
traj_o.L_br  = cat(3,traj.L_br(:,:,1:k_now-1),traj_t.L_br);

dt_fmu = 1/traj.hz;
N = size(traj_o.x_br,2);
traj_o.hz = traj.hz;
traj_o.type = traj.type;
traj_o.t_fmu = 0:dt_fmu:((N-1)*dt_fmu);

end