function plot_ukf_hist(sv, flight)
    b_single_ang_err = true; % set to false to plot euler angles, true to plot quat_dist angle
    msk = sv.hist_mask;
    disp('')
    
    
    figure(1234); clf; sgtitle("UKF - red     Ground Truth - blue")
    subplot(4,2,1); ylabel('X [m]'); hold on; grid on; xlabel('time (s)'); 
    
    % Positions
    max_disp = ceil(max(max([sv.mu_hist(1:3, msk), flight.x_act(1:3, :)]')));
    min_disp = floor(min(min([sv.mu_hist(1:3, msk), flight.x_act(1:3, :)]')));
    
    plot(sv.time_hist(msk), sv.mu_hist(1, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(1, :),'b-', 'LineWidth', 2)
    plot(sv.time_hist(msk), sv.dyn_ol_hist(1, 1:min(sv.do_ind-1,length(sv.time_hist(msk)))),'c-'); 
    ylim([min_disp, max_disp])
    
    subplot(4,2,3); ylabel('Y [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(2, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(2, :),'b-', 'LineWidth', 2)
    plot(sv.time_hist(msk), sv.dyn_ol_hist(2, 1:min(sv.do_ind-1,length(sv.time_hist(msk)))),'c-'); 
    ylim([min_disp, max_disp])
    
    subplot(4,2,5); ylabel('Z [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(3, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(3, :),'b-', 'LineWidth', 2)
    plot(sv.time_hist(msk), sv.dyn_ol_hist(3, 1:min(sv.do_ind-1,length(sv.time_hist(msk)))),'c-'); 
    ylim([min_disp, max_disp])
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Trace(Covar)
    subplot(4,2,7); ylabel('trace(covar)'); hold on; grid on; xlabel('time (s)');
%     title("Uncertainty vs Time")
    plot(sv.time_hist(msk), sv.sig_trace_hist(msk), 'm-')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Angles    
    max_ang = ceil(max(max([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
    min_ang = floor(min(min([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
    
    subplot(4,2,2); ylabel('yaw (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(1, msk),'r-', 'LineWidth', 2); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(1, msk),'b-', 'LineWidth', 2)
    ylim([min_ang, max_ang])

    subplot(4,2,4); ylabel('pitch (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), abs(sv.ypr_hist(2, msk)),'r-', 'LineWidth', 2); 
    plot(sv.time_hist(msk), abs(sv.ypr_act_hist(2, msk)),'b-', 'LineWidth', 2)
    ylim([min_ang, max_ang])

    subplot(4,2,6); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'r-', 'LineWidth', 2); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'b-', 'LineWidth', 2)
    ylim([min_ang, max_ang])
    
    subplot(4,2,8); ylabel('|angle error (deg)|'); hold on; grid on; xlabel('time (s)');
    plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     if(b_single_ang_err)
%         subplot(3,2,2); ylabel('|axis-angle angle (deg)|'); hold on; grid on; xlabel('time (s)');
%         plot(sv.time_hist(msk), sv.ang(msk),'r-', 'LineWidth', 2); 
%         plot(sv.time_hist(msk), sv.ang_act(msk),'b-', 'LineWidth', 2)
%         plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
%         
%         subplot(3,2,4); ylabel('|angle error (deg)| (zoomed)'); hold on; grid on; xlabel('time (s)');
%         plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
%         
%         subplot(3,2,6); ylabel('trace(covar)'); hold on; grid on; xlabel('time (s)');
%         title("Uncertainty vs Time")
%         plot(sv.time_hist(msk), sv.sig_trace_hist(msk), 'm-')
%     
%         figure(11541); clf; hold on %%%%%%%%%%%%%%%%%%
%         max_ang = ceil(max(max([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
%         min_ang = floor(min(min([sv.ypr_hist(:, msk), sv.ypr_act_hist(:, msk)]')));
% 
%         subplot(3,1,1); ylabel('yaw (deg)'); hold on; grid on; xlabel('time (s)'); 
%         plot(sv.time_hist(msk), sv.ypr_hist(1, msk),'r-', 'LineWidth', 2); 
%         plot(sv.time_hist(msk), sv.ypr_act_hist(1, msk),'b-', 'LineWidth', 2)
%         ylim([min_ang, max_ang])
% 
%         subplot(3,1,2); ylabel('pitch (deg)'); hold on; grid on; xlabel('time (s)'); 
%         plot(sv.time_hist(msk), sv.ypr_hist(2, msk),'r-', 'LineWidth', 2); 
%         plot(sv.time_hist(msk), sv.ypr_act_hist(2, msk),'b-', 'LineWidth', 2)
%         ylim([min_ang, max_ang])
% 
%         subplot(3,1,3); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
%         plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'r-', 'LineWidth', 2); 
%         plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'b-', 'LineWidth', 2)
%         ylim([min_ang, max_ang])
%         sgtitle("UKF - red     Ground Truth - blue")
%     end
    
    disp('')
end