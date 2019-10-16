function plot_acc_and_pitch(flight, sv, t_rbg_arr)
    %%%% PITCH DIR EST %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    acc = (flight.x_act(4, 2:end) - flight.x_act(4, 1:end-1))./(t_rbg_arr(2:end) - t_rbg_arr(1:end-1));
    figure(1246436); clf; sgtitle("Pitch vs calc. accel For GT")
    subplot(4,1,1); 
    hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(sv.hist_mask), flight.x_act(1, :),'b-', 'LineWidth', 2);  ylabel('x');
    
    subplot(4,1,2); 
    hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(sv.hist_mask), flight.x_act(4, :),'b-', 'LineWidth', 2);   ylabel('v_x');
    ylim([-5, 5])
    
    subplot(4,1,3); 
    hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(sv.hist_mask), [acc(1, :),acc(1, end)],'b-', 'LineWidth', 2);   ylabel('a_x');
    ylim([-65, 65])
    
    subplot(4,1,4); 
    hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(sv.hist_mask), sv.ypr_act_hist(2, sv.hist_mask),'r-', 'LineWidth', 2);   ylabel('pitch');
    ylim([-35, 35])
    
end
