function ukf_state_plot(sv, flight)
    msk = sv.hist_mask;
    disp('')
    figure(1234); clf; sgtitle("UKF - red     Ground Truth - blue")
    subplot(3,2,1); ylabel('X [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(1, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(1, msk),'bs')
    
    subplot(3,2,3); ylabel('Y [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(2, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(2, msk),'bs')
    
    subplot(3,2,5); ylabel('Z [m]'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.mu_hist(3, msk),'rs'); 
    plot(sv.time_hist(msk), flight.x_act(3, msk),'bs')
    
    subplot(3,2,2); ylabel('yaw (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(1, msk),'rs'); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(1, msk),'bs')
    
    subplot(3,2,4); ylabel('pitch (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(2, msk),'rs'); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(2, msk),'bs')
    
    subplot(3,2,6); ylabel('roll (deg)'); hold on; grid on; xlabel('time (s)'); 
    plot(sv.time_hist(msk), sv.ypr_hist(3, msk),'rs'); 
    plot(sv.time_hist(msk), sv.ypr_act_hist(3, msk),'bs')
%     sgtitle("UKF - red     Ground Truth - blue")
    
    
    disp('')
%     figure(111); clf; plot(sv.time_hist(msk), sv.sig_trace_hist(msk),'rs'); grid on; xlabel('time (s)'); ylabel('trace(Sigma)')
%     figure(222); clf; plot(sv.time_hist(msk), sv.mu_hist(1, msk),'rs'); grid on; xlabel('time (s)'); ylabel('quad state (x)'); hold on; 
%                       plot(sv.time_hist(msk), flight.x_act(1, msk),'bs')
%     figure(333); clf; plot(sv.time_hist(msk), sv.mu_hist(2, msk),'rs'); grid on; xlabel('time (s)'); ylabel('quad state (y)'); hold on; 
%                       plot(sv.time_hist(msk), flight.x_act(2, msk),'bs')
%     figure(444); clf; plot(sv.time_hist(msk), sv.mu_hist(3, msk),'rs'); grid on; xlabel('time (s)'); ylabel('quad state (z)'); hold on; 
%                       plot(sv.time_hist(msk), flight.x_act(3, msk),'bs')
    
%     disp('')
end