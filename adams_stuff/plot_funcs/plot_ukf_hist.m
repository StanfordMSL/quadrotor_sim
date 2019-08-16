function plot_ukf_hist(sv, flight)
    b_single_ang_err = true; % set to false to plot euler angles, true to plot quat_dist angle
    msk = sv.hist_mask;
    disp('')
    
    x_act_permuted = [flight.x_act(1:6, :); flight.x_act(10:12, :); flight.x_act(7:9, :)];
    
    max_disp = ceil(max(max([sv.mu_hist(1:3, msk), x_act_permuted(1:3, :)]')));
    min_disp = floor(min(min([sv.mu_hist(1:3, msk), x_act_permuted(1:3, :)]')));
    
    figure(1234); clf; sgtitle("UKF - red     Ground Truth - blue")
    subplot(3,2,1); ylabel('X [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(1, msk),'rs'); 
    plot(flight.t_act, x_act_permuted(1, :),'bs')
    ylim([min_disp, max_disp])
    
    subplot(3,2,3); ylabel('Y [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(2, msk),'rs'); 
    plot(flight.t_act, x_act_permuted(2, :),'bs')
    ylim([min_disp, max_disp])
    
    subplot(3,2,5); ylabel('Z [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(3, msk),'rs'); 
    plot(flight.t_act, x_act_permuted(3, :),'bs')
    ylim([min_disp, max_disp])
    
    if(b_single_ang_err)
        subplot(3,2,2); ylabel('|axis-angle angle (deg)|'); hold on; grid on; xlabel('time (s)');
        plot(sv.time_hist(msk), sv.ang(msk),'rs'); 
        plot(sv.time_hist(msk), sv.ang_act(msk),'bs')
        plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
        
        subplot(3,2,4); ylabel('|angle error (deg)| (zoomed)'); hold on; grid on; xlabel('time (s)');
        plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
        
        subplot(3,2,6); ylabel('trace(covar)'); hold on; grid on; xlabel('time (s)');
        title("Uncertainty vs Time")
        plot(sv.time_hist(msk), sv.sig_trace_hist(msk), 'm-')
    
        figure(11541); clf; hold on %%%%%%%%%%%%%%%%%%
        max_ang = ceil(max(max([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
        min_ang = floor(min(min([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));

        subplot(3,1,1); ylabel('yaw (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(1, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(1, msk),'bs')
        ylim([min_ang, max_ang])

        subplot(3,1,2); ylabel('pitch (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(2, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(2, msk),'bs')
        ylim([min_ang, max_ang])

        subplot(3,1,3); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
        plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'rs'); 
        plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'bs')
        ylim([min_ang, max_ang])
        sgtitle("UKF - red     Ground Truth - blue")
    end
    
    disp('')
end