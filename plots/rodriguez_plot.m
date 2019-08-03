function rod = rodriguez_plot(time,states,quat,mu_ekf,mu_ukf)
    mu_act      = zeros(13,time.fc_step_total);
    mu_act(:,1) = [states(1:6,1) ; quat(:,1); states(10:12,1)];
    
    for k = 2:time.fc_step_total        
        index = round(k*time.fc_dt/time.act_dt);
        if index > time.act_step_total
            index = time.act_step_total;
        end
        mu_act(:,k)  = [states(1:6,index) ; quat(:,index); states(10:12,index)];
    end
    
    rod.ekf = zeros(4,time.fc_step_total);
    rod.iekf = zeros(4,time.fc_step_total);
    rod.ukf = zeros(4,time.fc_step_total);
    for k = 1:time.fc_step_total
        R_act  = quat2rotm(mu_act(7:10,k)');
        R_ekf  = quat2rotm(mu_ekf(7:10,k)');
        R_ukf  = quat2rotm(mu_ukf(7:10,k)');
        
        rod.ekf(1:3,k)  = rotationMatrixToVector(R_ekf*R_act');
        rod.ukf(1:3,k)  = rotationMatrixToVector(R_ukf*R_act');
        
        rod.ekf(4,k)  = norm(rod.ekf(1:3,k));
        rod.ukf(4,k)  = norm(rod.ukf(1:3,k));
    end
    
    plot(time.t_fc,rod.ekf(4,:),time.t_fc,rod.ukf(4,:));
    title('Rod. \phi (rad)');
    legend('EKF','UKF');
end
