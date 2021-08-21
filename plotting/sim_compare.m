function sim_compare(traj,log_M,log_R)

n = 34;

figure(4)
clf

title_arr = {'p_x','p_y','p_z','v_x','v_y','v_z','q_w','q_x','q_y','q_z'};
N = size(title_arr,2);
R = 6;
C = 2;
for k = 1:N
    if k <= R
        idx = 2*k-1;
    else
        idx = (k-R)*2;
    end
    
    subplot(R,C,idx)
    plot(traj.t_fmu,traj.x_bar(k,:),'--','Linewidth',1.5);
    hold on
    plot(log_M.t_fmu,log_M.x_fmu(k,:));
    plot(log_R.t_fmu(1,:),log_R.x_fmu(k,:));
    title(title_arr{k});
    legend('nominal','matlab','ros');
    
    if k <= R
        ylim([-2.5 2.5]);
    else
        ylim([-1 1]);
    end
end

% figure(6)
% clf
% 
% title_arr = {'thrust','w_x','w_y','w_z'};
% N = size(title_arr,2);
% for k = 1:N
%     subplot(N,1,k)
%     
%     plot(traj.t_fmu(1:end-1),traj.u_br(k,:),'--','Linewidth',1.5);
%     hold on
%     plot(log_M.t_fmu(1:end-1),log_M.u_br(k,:));
%     plot(log_R.t_fmu(1,1:58),log_R.u_fmu(k,1:58));
%     
%     title(title_arr{k});
%     legend('nominal','matlab','ros');
%     
%     if k == 1
%         ylim([0 1]);
%     else
%         ylim([-2.5 2.5]);
%     end
% end

end