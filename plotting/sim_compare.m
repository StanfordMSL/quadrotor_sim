function sim_compare(traj,log_M,log_R)

figure(4)
clf
set(gcf,'color','w');

title_arr = {'p_x','p_y','p_z','v_x','v_y','v_z'};
N = size(title_arr,2);
for k = 1:N
    subplot(N,1,k)
    plot(traj.t_fmu,traj.x_bar(k,:));
    hold on
    plot(log_M.t_fmu,log_M.x_fmu(k,:));
    plot(log_R.t_fmu,log_R.x_fmu(k,:));
    legend('nominal','matlab','ros');
    title(title_arr{k});
end

figure(5)
clf
set(gcf,'color','w');

title_arr = {'q_w','q_x','q_y','q_z','w_x','w_y','w_z'};
N = size(title_arr,2);
for k = 1:N
    subplot(N,1,k)
    
    idx = k+6;
    plot(traj.t_fmu,traj.x_bar(idx,:));
    hold on
    plot(log_M.t_fmu,log_M.x_fmu(idx,:));
    plot(log_R.t_fmu,log_R.x_fmu(idx,:));
    legend('nominal','matlab','ros');
    
    title(title_arr{k});
end

figure(6)
clf
set(gcf,'color','w');

title_arr = {'thrust','w_x','w_y','w_z'};
N = size(title_arr,2);
for k = 1:N
    subplot(N,1,k)
    
    plot(traj.t_fmu,traj.u_br(idx,:));
    hold on
    plot(log_M.t_fmu,log_M.u_br(idx,:));
    plot(log_R.t_fmu,log_R.u_fmu(idx,:));
    legend('nominal','matlab','ros');
    
    title(title_arr{k});
end

end