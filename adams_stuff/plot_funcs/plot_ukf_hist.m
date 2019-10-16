function plot_ukf_hist(sv, flight)
    b_single_ang_err = true; % set to false to plot euler angles, true to plot quat_dist angle
    msk = sv.hist_mask;
    disp('')
    
    
    figure(1234); clf; sgtitle("Poses: UKF - red     Ground Truth - blue")
    subplot(4,2,1); ylabel('X [m]'); hold on; grid on; xlabel('time (s)'); 
    
    % Positions
    max_disp = ceil(max(max([sv.mu_hist(1:3, msk), flight.x_act(1:3, :)]')));
    min_disp = floor(min(min([sv.mu_hist(1:3, msk), flight.x_act(1:3, :)]')));
    
    plot(sv.time_hist(msk), sv.mu_hist(1, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(1, :),'b-', 'LineWidth', 2)
    ylim([min_disp, max_disp])
    
    subplot(4,2,3); ylabel('Y [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(2, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(2, :),'b-', 'LineWidth', 2)
    ylim([min_disp, max_disp])
    
    subplot(4,2,5); ylabel('Z [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(3, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(3, :),'b-', 'LineWidth', 2)
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
    plot(sv.time_hist(msk), sv.ypr_hist(2, msk),'r-', 'LineWidth', 2); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(2, msk),'b-', 'LineWidth', 2)
    ylim([min_ang, max_ang])

    subplot(4,2,6); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'r-', 'LineWidth', 2); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'b-', 'LineWidth', 2)
    ylim([min_ang, max_ang])
    
    subplot(4,2,8); ylabel('|angle error (deg)|'); hold on; grid on; xlabel('time (s)');
    plot(sv.time_hist(msk), sv.ang_err(msk),'m-'); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    figure(1236); clf; sgtitle("Velocities: UKF - red     Ground Truth - blue")
    subplot(3,2,1); ylabel('v_x [m/s]'); hold on; grid on; xlabel('time (s)'); 
    
    max_vel = ceil(max(max([sv.mu_hist(4:6, msk), flight.x_act(4:6, :)]')));
    min_vel = floor(min(min([sv.mu_hist(4:6, msk), flight.x_act(4:6, :)]')));
    max_ang_vel = ceil(max(max([sv.mu_hist(11:13, msk), flight.x_act(11:13, :)]')));
    min_ang_vel = floor(min(min([sv.mu_hist(11:13, msk), flight.x_act(11:13, :)]')));
    
    plot(sv.time_hist(msk), sv.mu_hist(4, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(4, :),'b-', 'LineWidth', 2); 
%     ylim([min_vel, max_vel])
    
    subplot(3,2,3); ylabel('v_y [m/s]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(5, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(5, :),'b-', 'LineWidth', 2)
%     ylim([min_vel, max_vel])
    
    subplot(3,2,5); ylabel('v_z [m/s]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(6, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(6, :),'b-', 'LineWidth', 2)
%     ylim([min_vel, max_vel])

    subplot(3,2,2); ylabel('\omega_x [rad/s]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(11, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(11, :),'b-', 'LineWidth', 2); 
%     ylim([min_ang_vel, max_ang_vel])
    
    subplot(3,2,4); ylabel('\omega_y [rad/s]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(12, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(12, :),'b-', 'LineWidth', 2)
%     ylim([min_ang_vel, max_ang_vel])
    
    subplot(3,2,6); ylabel('\omega_z [rad/s]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(13, msk),'r-', 'LineWidth', 2); 
    plot(flight.t_act, flight.x_act(13, :),'b-', 'LineWidth', 2)
%     ylim([min_ang_vel, max_ang_vel])

    disp('')
end
