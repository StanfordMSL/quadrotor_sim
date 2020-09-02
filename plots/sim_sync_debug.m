function sim_sync_debug(log,traj)

figure(4)
clf

for k = 1:3
    plot(log.x_est(k,:),'r');
    hold on
    plot(traj.x(k,:),'b');
    hold on
end