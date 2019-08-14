function ukf_state_plot(sv, flight)
    b_single_ang_err = true; % set to false to plot euler angles, true to plot quat_dist angle
    msk = sv.hist_mask;
    disp('')
    
    max_disp = ceil(max(max([sv.mu_hist(1:3, msk), flight.x_act(1:3, msk)]')));
    min_disp = floor(min(min([sv.mu_hist(1:3, msk), flight.x_act(1:3, msk)]')));
    
    figure(1234); clf; sgtitle("UKF - red     Ground Truth - blue")
    subplot(3,2,1); ylabel('X [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(1, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(1, msk),'bs')
    ylim([min_disp, max_disp])
    
    subplot(3,2,3); ylabel('Y [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(2, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(2, msk),'bs')
    ylim([min_disp, max_disp])
    
    subplot(3,2,5); ylabel('Z [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(3, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(3, msk),'bs')
    ylim([min_disp, max_disp])
    
    if(b_single_ang_err)
        subplot(3,2,2); ylabel('axis-angle angle (deg)'); hold on; grid on; xlabel('time (s)');
        plot(sv.time_hist(msk), sv.ang(msk)*180/pi,'rs'); 
        plot(sv.time_hist(msk), sv.ang_act(msk)*180/pi,'bs')
        
        subplot(3,2,4); ylabel('|angle error (deg)|'); hold on; grid on; xlabel('time (s)');
        plot(sv.time_hist(msk), sv.ang_err(msk),'ms'); 
        
        subplot(3,2,6); ylabel('trace(covar)'); hold on; grid on; xlabel('time (s)');
        title("Uncertainty vs Time")
        plot(sv.time_hist(msk), sv.sig_trace_hist(msk), 'ms')
    else
        max_ang = ceil(max(max([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
        min_ang = floor(min(min([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));

        subplot(3,2,2); ylabel('yaw (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(1, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(1, msk),'bs')
        ylim([min_ang, max_ang])

        subplot(3,2,4); ylabel('pitch (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(2, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(2, msk),'bs')
        ylim([min_ang, max_ang])

        subplot(3,2,6); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'bs')
        ylim([min_ang, max_ang])
        sgtitle("UKF - red     Ground Truth - blue")

        figure(23213); clf; ylabel('trace(covar)'); hold on; grid on; xlabel('time (s)'); 
        title("Uncertainty vs Time")
        plot(sv.time_hist(msk), sv.sig_trace_hist(msk), 'ms')
    end
    
    disp('')
end