function [x_est, sigma_est] = ekf(x_curr,sigma_curr,u_now,y,model)
    % Unpack some stuff
    dt_fmu = model.dt_fmu;
    C = model.C;
    Q = model.Q_fil;
    R = model.R_fil;
    
    % Predict Forward Dynamics
    x_pred = quadcopter(x_curr,u_now,model,zeros(6,1),'fmu_ideal');
    
    % Predict the Covariance
    [A,~] = stagewise_linearizer(x_curr,u_now,dt_fmu);
      
    sig_pred = A*sigma_curr*A'+Q;

    % Update 
    g_ekf = sensors(x_pred,model);
    
    K = sig_pred*C'/(C*sig_pred*C'+R);
    
    x_est    = x_pred  + K*(y-g_ekf);
    
    x_est(7:10,1) = x_est(7:10,1)./norm(x_est(7:10,1));
    sigma_est = sig_pred - K*C*sig_pred;
end