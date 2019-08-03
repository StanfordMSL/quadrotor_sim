function [err_ekf, err_iekf, err_ukf] = error_analysis(time,x,quat,mu_ekf,mu_iekf,mu_ukf)
    x_act  = zeros(4,time.fc_step_total);
    x_act(:,1)  = quat(:,1);
    
    for k = 2:time.fc_step_total        
        index = round(k*time.fc_dt/time.act_dt);
        if index > time.act_step_total
            index = time.act_step_total;
        end
        x_act(:,k)  = quat(:,index);
    end
    
    err_ekf  = abs(mu_ekf(7:10,:)  - x_act);
    err_iekf = abs(mu_iekf(7:10,:) - x_act);
    err_ukf  = abs(mu_ukf(7:10,:)  - x_act);
    
%     norm_ekf_error  = norm(err_ekf);
%     norm_iekf_error = norm(err_iekf);
%     norm_ukf_error  = norm(err_ukf);
%     
%     total_ekf_error  = sum(norm_ekf_error);
%     total_iekf_error = sum(norm_iekf_error);
%     total_ukf_error  = sum(norm_ukf_error);
%     
%     text_ekf  = strcat('EKF Quat error: ' ,num2str(total_ekf_error));
%     text_iekf = strcat('iEKF Quat error: ',num2str(total_iekf_error));
%     text_ukf  = strcat('UKF Quat error: ' ,num2str(total_ukf_error));
%     disp(text_ekf);
%     disp(text_iekf);
%     disp(text_ukf);
end