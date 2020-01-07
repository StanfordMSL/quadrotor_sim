function dist = distance_plot(time,states,quat,mu_ekf,mu_ukf)
    mu_act      = zeros(13,time.fc_step_total);
    mu_act(:,1) = [states(1:6,1) ; quat(:,1); states(11:13,1)];
    
    for k = 2:time.fc_step_total        
        index = round(k*time.fc_dt/time.act_dt);
        if index > time.act_step_total
            index = time.act_step_total;
        end
        mu_act(:,k)  = [states(1:6,index) ; quat(:,index); states(11:13,index)];
    end
    
    dist.ekf  = zeros(1,time.fc_step_total);
    dist.ukf  = zeros(1,time.fc_step_total);
    for k = 1:time.fc_step_total
        dist.ekf(1,k)  = norm(mu_act(1:3,k)-mu_ekf(1:3,k));
        dist.ukf(1,k)  = norm(mu_act(1:3,k)-mu_ukf(1:3,k));
    end
  
    plot(time.t_fc,dist.ekf,time.t_fc,dist.ukf);
    title('Distance Error (m)');
    legend('EKF','UKF')
end
