function sim_sync_debug(log,traj)

figure(4)
clf

for k = 1:3
    plot(log.x_est(k,:));
    hold on
    plot(traj.x_bar(k,:));
    hold on
end